import argparse
import numpy as np
import csv
from pathlib import Path
from scipy.spatial.transform import Rotation

def load_trajectory_csv(filepath):
    timestamps = []
    positions = []
    quaternions = []
    
    try:
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                if len(row) >= 8:
                    timestamps.append(float(row[0]))
                    positions.append([float(row[1]), float(row[2]), float(row[3])])
                    quaternions.append([float(row[4]), float(row[5]), float(row[6]), float(row[7])])
    except Exception as e:
        print(f"Error reading trajectory file: {e}")
        return None, None, None
    
    if not timestamps:
        return None, None, None
    
    return np.array(timestamps), np.array(positions), np.array(quaternions)

def load_kitti_ground_truth(filepath):
    poses = []
    try:
        with open(filepath, 'r') as f:
            for line in f:
                values = [float(v) for v in line.strip().split()]
                if len(values) >= 12:
                    pose = np.eye(4)
                    pose[0:3, 0:4] = np.array(values).reshape(3, 4)
                    poses.append(pose)
    except Exception as e:
        print(f"Error reading KITTI ground truth: {e}")
        return None
    
    return poses

def compute_absolute_pose_error(estimated, ground_truth):
    if len(estimated) != len(ground_truth):
        min_len = min(len(estimated), len(ground_truth))
        estimated = estimated[:min_len]
        ground_truth = ground_truth[:min_len]
    
    errors = []
    for est, gt in zip(estimated, ground_truth):
        if est.shape == (4, 4) and gt.shape == (4, 4):
            error = np.linalg.norm(est[0:3, 3] - gt[0:3, 3])
            errors.append(error)
    
    if not errors:
        return None, None, None
    
    errors = np.array(errors)
    return np.mean(errors), np.sqrt(np.mean(errors ** 2)), np.max(errors)

def compute_relative_pose_error(trajectory1, trajectory2, distance_threshold=1.0):
    if len(trajectory1) < 2 or len(trajectory2) < 2:
        return None, None, None
    
    rpe_trans = []
    rpe_rot = []
    
    for i in range(len(trajectory1) - 1):
        if i >= len(trajectory2) - 1:
            break
        
        pose_a1 = trajectory1[i]
        pose_a2 = trajectory1[i + 1]
        pose_b1 = trajectory2[i]
        pose_b2 = trajectory2[i + 1]
        
        delta_a = np.linalg.inv(pose_a1) @ pose_a2
        delta_b = np.linalg.inv(pose_b1) @ pose_b2
        
        trans_error = np.linalg.norm(delta_a[0:3, 3] - delta_b[0:3, 3])
        
        rot_a = Rotation.from_matrix(delta_a[0:3, 0:3])
        rot_b = Rotation.from_matrix(delta_b[0:3, 0:3])
        rot_error = Rotation.from_matrix((rot_a.inv() * rot_b).as_matrix()).magnitude()
        
        rpe_trans.append(trans_error)
        rpe_rot.append(np.degrees(rot_error))
    
    if not rpe_trans:
        return None, None, None
    
    rpe_trans = np.array(rpe_trans)
    rpe_rot = np.array(rpe_rot)
    
    mean_trans = np.mean(rpe_trans)
    mean_rot = np.mean(rpe_rot)
    rmse_trans = np.sqrt(np.mean(rpe_trans ** 2))
    
    return mean_trans, rmse_trans, mean_rot

def format_trajectory_for_evo(trajectory_csv, output_file):
    timestamps, positions, quaternions = load_trajectory_csv(trajectory_csv)
    
    if timestamps is None:
        print("Failed to load trajectory")
        return False
    
    try:
        with open(output_file, 'w') as f:
            for ts, pos, quat in zip(timestamps, positions, quaternions):
                line = f"{ts} {pos[0]} {pos[1]} {pos[2]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}\n"
                f.write(line)
        print(f"Formatted trajectory saved to {output_file}")
        return True
    except Exception as e:
        print(f"Error writing formatted trajectory: {e}")
        return False

def evaluate_trajectory(estimated_csv, ground_truth_file, output_dir=None):
    print("Loading trajectories...")
    _, est_positions, _ = load_trajectory_csv(estimated_csv)
    
    if est_positions is None:
        print("Failed to load estimated trajectory")
        return
    
    gt_poses = load_kitti_ground_truth(ground_truth_file)
    
    if gt_poses is None:
        print("Failed to load ground truth")
        return
    
    if len(est_positions) > len(gt_poses):
        est_positions = est_positions[:len(gt_poses)]
    else:
        gt_poses = gt_poses[:len(est_positions)]
    
    estimated_poses = []
    for pos in est_positions:
        pose = np.eye(4)
        pose[0:3, 3] = pos
        estimated_poses.append(pose)
    
    print("\n=== Absolute Pose Error (APE) ===")
    mean_ape, rmse_ape, max_ape = compute_absolute_pose_error(estimated_poses, gt_poses)
    
    if mean_ape is not None:
        print(f"Mean APE: {mean_ape:.6f} m")
        print(f"RMSE APE: {rmse_ape:.6f} m")
        print(f"Max APE: {max_ape:.6f} m")
    
    print("\n=== Relative Pose Error (RPE) ===")
    mean_rpe_trans, rmse_rpe_trans, mean_rpe_rot = compute_relative_pose_error(
        estimated_poses, gt_poses)
    
    if mean_rpe_trans is not None:
        print(f"Mean RPE Translation: {mean_rpe_trans:.6f} m")
        print(f"RMSE RPE Translation: {rmse_rpe_trans:.6f} m")
        print(f"Mean RPE Rotation: {mean_rpe_rot:.4f} degrees")
    
    if output_dir:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        results_file = output_path / "benchmark_results.txt"
        with open(results_file, 'w') as f:
            f.write("=== Absolute Pose Error (APE) ===\n")
            if mean_ape is not None:
                f.write(f"Mean APE: {mean_ape:.6f} m\n")
                f.write(f"RMSE APE: {rmse_ape:.6f} m\n")
                f.write(f"Max APE: {max_ape:.6f} m\n")
            
            f.write("\n=== Relative Pose Error (RPE) ===\n")
            if mean_rpe_trans is not None:
                f.write(f"Mean RPE Translation: {mean_rpe_trans:.6f} m\n")
                f.write(f"RMSE RPE Translation: {rmse_rpe_trans:.6f} m\n")
                f.write(f"Mean RPE Rotation: {mean_rpe_rot:.4f} degrees\n")
        
        print(f"\nResults saved to {results_file}")

def main():
    parser = argparse.ArgumentParser(
        description='Evaluate SLAM trajectory against ground truth')
    parser.add_argument('estimated_trajectory',
                       help='Estimated trajectory CSV file')
    parser.add_argument('ground_truth', help='Ground truth trajectory file')
    parser.add_argument('--output-dir', help='Output directory for results')
    parser.add_argument('--format-for-evo', help='Format trajectory for evo tool')
    
    args = parser.parse_args()
    
    if args.format_for_evo:
        format_trajectory_for_evo(args.estimated_trajectory, args.format_for_evo)
    else:
        evaluate_trajectory(args.estimated_trajectory, args.ground_truth,
                           args.output_dir)

if __name__ == '__main__':
    main()
