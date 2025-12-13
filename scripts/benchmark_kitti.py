import argparse
import subprocess
import sys
from pathlib import Path

def play_rosbag_dataset(bag_file, sim_time=True):
    cmd = ['ros2', 'bag', 'play', bag_file]
    if sim_time:
        cmd.extend(['--clock'])
    
    print(f"Playing rosbag: {bag_file}")
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error playing rosbag: {e}")
        return False
    except FileNotFoundError:
        print("ros2 bag command not found. Make sure ROS2 is installed.")
        return False
    
    return True

def extract_kitti_sequence(kitti_root, sequence_id):
    sequence_dir = Path(kitti_root) / f"{sequence_id:02d}"
    
    if not sequence_dir.exists():
        print(f"KITTI sequence directory not found: {sequence_dir}")
        return None
    
    image_files = sorted(sequence_dir.glob("image_0/*.png"))
    lidar_files = sorted(sequence_dir.glob("velodyne/*.bin"))
    
    if not image_files or not lidar_files:
        print(f"Missing image or lidar files in sequence {sequence_id:02d}")
        return None
    
    ground_truth_file = Path(kitti_root).parent / "poses" / f"{sequence_id:02d}.txt"
    if not ground_truth_file.exists():
        print(f"Ground truth file not found: {ground_truth_file}")
        return None
    
    return {
        'images': image_files,
        'lidar': lidar_files,
        'ground_truth': str(ground_truth_file),
        'sequence': sequence_id
    }

def run_benchmark(benchmark_script, estimated_trajectory, ground_truth, output_dir=None):
    cmd = [sys.executable, benchmark_script, estimated_trajectory, ground_truth]
    
    if output_dir:
        cmd.extend(['--output-dir', output_dir])
    
    print(f"Running benchmark...")
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Benchmark execution failed: {e}")
        return False
    
    return True

def main():
    parser = argparse.ArgumentParser(
        description='SLAM benchmark utilities for KITTI and TUM datasets')
    
    subparsers = parser.add_subparsers(dest='command', help='Command to run')
    
    rosbag_parser = subparsers.add_parser('play-bag', help='Play a ROS2 bag file')
    rosbag_parser.add_argument('bag_file', help='Path to ROS2 bag file')
    rosbag_parser.add_argument('--sim-time', action='store_true', default=True,
                              help='Use simulation time')
    
    kitti_parser = subparsers.add_parser('kitti-sequence',
                                        help='Extract KITTI sequence info')
    kitti_parser.add_argument('kitti_root', help='KITTI dataset root directory')
    kitti_parser.add_argument('--sequence', type=int, default=0,
                             help='Sequence ID (0-20)')
    
    benchmark_parser = subparsers.add_parser('evaluate',
                                            help='Evaluate trajectory')
    benchmark_parser.add_argument('estimated_trajectory',
                                 help='Estimated trajectory file')
    benchmark_parser.add_argument('ground_truth', help='Ground truth file')
    benchmark_parser.add_argument('--output-dir', help='Output directory')
    
    args = parser.parse_args()
    
    if args.command == 'play-bag':
        success = play_rosbag_dataset(args.bag_file, args.sim_time)
        sys.exit(0 if success else 1)
    
    elif args.command == 'kitti-sequence':
        info = extract_kitti_sequence(args.kitti_root, args.sequence)
        if info:
            print(f"Sequence {info['sequence']:02d}:")
            print(f"  Images: {len(info['images'])} frames")
            print(f"  LiDAR: {len(info['lidar'])} scans")
            print(f"  Ground truth: {info['ground_truth']}")
        else:
            sys.exit(1)
    
    elif args.command == 'evaluate':
        script_dir = Path(__file__).parent
        evaluate_script = script_dir / "evaluate_trajectory.py"
        success = run_benchmark(str(evaluate_script), args.estimated_trajectory,
                               args.ground_truth, args.output_dir)
        sys.exit(0 if success else 1)
    
    else:
        parser.print_help()

if __name__ == '__main__':
    main()
