import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import argparse
import seaborn as sns

def parse_arguments():
    parser = argparse.ArgumentParser(description='Compare MPU sensor data: raw vs Kalman and between sensors')
    parser.add_argument('--raw-file', default='mpu_raw_log_together.csv', 
                       help='Path to the raw data CSV file')
    parser.add_argument('--kalman-file', default='mpu_kalman_log_together.csv', 
                       help='Path to the Kalman filtered data CSV file')
    parser.add_argument('--start-time', default=None,
                       help='Start time for filtering data (format: YYYY-MM-DD HH:MM:SS)')
    parser.add_argument('--end-time', default=None,
                       help='End time for filtering data (format: YYYY-MM-DD HH:MM:SS)')
    parser.add_argument('--save-plots', action='store_true',
                       help='Save plots to files instead of showing them')
    return parser.parse_args()

def load_and_preprocess_data(file_path, start_time=None, end_time=None):
    """Load CSV data and preprocess it"""
    df = pd.read_csv(file_path)
    
    # Convert timestamp to datetime
    df['timestamp'] = pd.to_datetime(df['timestamp_iso'])
    
    # Filter by time if requested
    if start_time:
        start_dt = pd.to_datetime(start_time)
        df = df[df['timestamp'] >= start_dt]
    
    if end_time:
        end_dt = pd.to_datetime(end_time)
        df = df[df['timestamp'] <= end_dt]
    
    # Convert numeric columns
    numeric_cols = ['ts_epoch', 'dt_s', 'ax', 'ay', 'az', 'ax_g', 'ay_g', 'az_g', 
                    'gx', 'gy', 'gz', 'gx_dps', 'gy_dps', 'gz_dps', 'roll_acc', 'pitch_acc']
    
    # Add kalman columns if they exist
    if 'kalX_deg' in df.columns:
        numeric_cols.extend(['kalX_deg', 'kalY_deg'])
    
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    
    return df

def plot_raw_vs_kalman_comparison(raw_df, kalman_df, save_plots=False):
    """Create comparison plots between raw and Kalman-filtered data for both sensors"""
    
    # Separate data by sensor
    raw_mpu1 = raw_df[raw_df['mpu_num'] == 1]
    raw_mpu2 = raw_df[raw_df['mpu_num'] == 2]
    kalman_mpu1 = kalman_df[kalman_df['mpu_num'] == 1]
    kalman_mpu2 = kalman_df[kalman_df['mpu_num'] == 2]
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Raw vs Kalman-Filtered Data Comparison for Both Sensors', fontsize=16)
    
    # Plot 1: MPU1 Roll comparison
    axes[0, 0].plot(raw_mpu1['timestamp'], raw_mpu1['roll_acc'], 'r-', label='Raw Roll', alpha=0.7)
    axes[0, 0].plot(kalman_mpu1['timestamp'], kalman_mpu1['kalX_deg'], 'b-', label='Kalman Roll', alpha=0.7)
    axes[0, 0].set_title('MPU1: Roll Angle Comparison')
    axes[0, 0].set_ylabel('Angle (degrees)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].tick_params(axis='x', rotation=45)
    
    # Plot 2: MPU1 Pitch comparison
    axes[0, 1].plot(raw_mpu1['timestamp'], raw_mpu1['pitch_acc'], 'r-', label='Raw Pitch', alpha=0.7)
    axes[0, 1].plot(kalman_mpu1['timestamp'], kalman_mpu1['kalY_deg'], 'b-', label='Kalman Pitch', alpha=0.7)
    axes[0, 1].set_title('MPU1: Pitch Angle Comparison')
    axes[0, 1].set_ylabel('Angle (degrees)')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    axes[0, 1].tick_params(axis='x', rotation=45)
    
    # Plot 3: MPU2 Roll comparison
    axes[1, 0].plot(raw_mpu2['timestamp'], raw_mpu2['roll_acc'], 'r-', label='Raw Roll', alpha=0.7)
    axes[1, 0].plot(kalman_mpu2['timestamp'], kalman_mpu2['kalX_deg'], 'b-', label='Kalman Roll', alpha=0.7)
    axes[1, 0].set_title('MPU2: Roll Angle Comparison')
    axes[1, 0].set_ylabel('Angle (degrees)')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    axes[1, 0].tick_params(axis='x', rotation=45)
    
    # Plot 4: MPU2 Pitch comparison
    axes[1, 1].plot(raw_mpu2['timestamp'], raw_mpu2['pitch_acc'], 'r-', label='Raw Pitch', alpha=0.7)
    axes[1, 1].plot(kalman_mpu2['timestamp'], kalman_mpu2['kalY_deg'], 'b-', label='Kalman Pitch', alpha=0.7)
    axes[1, 1].set_title('MPU2: Pitch Angle Comparison')
    axes[1, 1].set_ylabel('Angle (degrees)')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    axes[1, 1].tick_params(axis='x', rotation=45)
    
    plt.tight_layout()
    
    if save_plots:
        plt.savefig('raw_vs_kalman_comparison.png', dpi=300, bbox_inches='tight')
        print("Plot saved as 'raw_vs_kalman_comparison.png'")
    else:
        plt.show()
    
    # Calculate and print statistics for raw vs Kalman differences
    print("\n=== RAW VS KALMAN DIFFERENCE STATISTICS ===")
    
    for sensor_num in [1, 2]:
        raw_data = raw_df[raw_df['mpu_num'] == sensor_num]
        kalman_data = kalman_df[kalman_df['mpu_num'] == sensor_num]
        
        # Merge data by timestamp
        merged = pd.merge_asof(
            raw_data[['timestamp', 'roll_acc', 'pitch_acc']].sort_values('timestamp'),
            kalman_data[['timestamp', 'kalX_deg', 'kalY_deg']].sort_values('timestamp'),
            on='timestamp'
        )
        
        merged['roll_error'] = merged['roll_acc'] - merged['kalX_deg']
        merged['pitch_error'] = merged['pitch_acc'] - merged['kalY_deg']
        
        print(f"\nMPU {sensor_num}:")
        print(f"  Roll Error - Mean: {merged['roll_error'].mean():.4f}°, Std: {merged['roll_error'].std():.4f}°")
        print(f"  Pitch Error - Mean: {merged['pitch_error'].mean():.4f}°, Std: {merged['pitch_error'].std():.4f}°")

def plot_sensor_differences(raw_df, kalman_df, save_plots=False):
    """Create plots showing differences between the two sensors for both raw and Kalman data"""
    
    # Separate data by sensor
    raw_mpu1 = raw_df[raw_df['mpu_num'] == 1]
    raw_mpu2 = raw_df[raw_df['mpu_num'] == 2]
    kalman_mpu1 = kalman_df[kalman_df['mpu_num'] == 1]
    kalman_mpu2 = kalman_df[kalman_df['mpu_num'] == 2]
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Differences Between MPU Sensors (Raw and Kalman)', fontsize=16)
    
    # Align data by timestamp for raw data
    raw_aligned = pd.merge_asof(
        raw_mpu1[['timestamp', 'roll_acc', 'pitch_acc']].sort_values('timestamp'),
        raw_mpu2[['timestamp', 'roll_acc', 'pitch_acc']].sort_values('timestamp'),
        on='timestamp', 
        suffixes=('_mpu1', '_mpu2')
    )
    
    # Align data by timestamp for Kalman data
    kalman_aligned = pd.merge_asof(
        kalman_mpu1[['timestamp', 'kalX_deg', 'kalY_deg']].sort_values('timestamp'),
        kalman_mpu2[['timestamp', 'kalX_deg', 'kalY_deg']].sort_values('timestamp'),
        on='timestamp', 
        suffixes=('_mpu1', '_mpu2')
    )
    
    # Calculate differences
    raw_aligned['roll_diff'] = raw_aligned['roll_acc_mpu1'] - raw_aligned['roll_acc_mpu2']
    raw_aligned['pitch_diff'] = raw_aligned['pitch_acc_mpu1'] - raw_aligned['pitch_acc_mpu2']
    kalman_aligned['roll_diff'] = kalman_aligned['kalX_deg_mpu1'] - kalman_aligned['kalX_deg_mpu2']
    kalman_aligned['pitch_diff'] = kalman_aligned['kalY_deg_mpu1'] - kalman_aligned['kalY_deg_mpu2']
    
    # Plot 1: Raw roll difference
    axes[0, 0].plot(raw_aligned['timestamp'], raw_aligned['roll_diff'], 'r-', alpha=0.7)
    axes[0, 0].set_title('Raw Data: Roll Difference (MPU1 - MPU2)')
    axes[0, 0].set_ylabel('Angle Difference (degrees)')
    axes[0, 0].grid(True)
    axes[0, 0].tick_params(axis='x', rotation=45)
    
    # Plot 2: Raw pitch difference
    axes[0, 1].plot(raw_aligned['timestamp'], raw_aligned['pitch_diff'], 'b-', alpha=0.7)
    axes[0, 1].set_title('Raw Data: Pitch Difference (MPU1 - MPU2)')
    axes[0, 1].set_ylabel('Angle Difference (degrees)')
    axes[0, 1].grid(True)
    axes[0, 1].tick_params(axis='x', rotation=45)
    
    # Plot 3: Kalman roll difference
    axes[1, 0].plot(kalman_aligned['timestamp'], kalman_aligned['roll_diff'], 'r-', alpha=0.7)
    axes[1, 0].set_title('Kalman Data: Roll Difference (MPU1 - MPU2)')
    axes[1, 0].set_ylabel('Angle Difference (degrees)')
    axes[1, 0].grid(True)
    axes[1, 0].tick_params(axis='x', rotation=45)
    
    # Plot 4: Kalman pitch difference
    axes[1, 1].plot(kalman_aligned['timestamp'], kalman_aligned['pitch_diff'], 'b-', alpha=0.7)
    axes[1, 1].set_title('Kalman Data: Pitch Difference (MPU1 - MPU2)')
    axes[1, 1].set_ylabel('Angle Difference (degrees)')
    axes[1, 1].grid(True)
    axes[1, 1].tick_params(axis='x', rotation=45)
    
    plt.tight_layout()
    
    if save_plots:
        plt.savefig('sensor_differences.png', dpi=300, bbox_inches='tight')
        print("Plot saved as 'sensor_differences.png'")
    else:
        plt.show()
    
    # Calculate and print statistics for sensor differences
    print("\n=== SENSOR DIFFERENCE STATISTICS ===")
    
    print("\nRaw Data Differences:")
    print(f"Roll Difference - Mean: {raw_aligned['roll_diff'].mean():.4f}°, Std: {raw_aligned['roll_diff'].std():.4f}°")
    print(f"Pitch Difference - Mean: {raw_aligned['pitch_diff'].mean():.4f}°, Std: {raw_aligned['pitch_diff'].std():.4f}°")
    
    print("\nKalman Data Differences:")
    print(f"Roll Difference - Mean: {kalman_aligned['roll_diff'].mean():.4f}°, Std: {kalman_aligned['roll_diff'].std():.4f}°")
    print(f"Pitch Difference - Mean: {kalman_aligned['pitch_diff'].mean():.4f}°, Std: {kalman_aligned['pitch_diff'].std():.4f}°")

def plot_error_distributions(raw_df, kalman_df, save_plots=False):
    """Plot distributions of errors for both raw and Kalman data"""
    
    # Separate data by sensor
    raw_mpu1 = raw_df[raw_df['mpu_num'] == 1]
    raw_mpu2 = raw_df[raw_df['mpu_num'] == 2]
    kalman_mpu1 = kalman_df[kalman_df['mpu_num'] == 1]
    kalman_mpu2 = kalman_df[kalman_df['mpu_num'] == 2]
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Error Distributions', fontsize=16)
    
    # Calculate raw vs Kalman errors for each sensor
    for sensor_num, color in zip([1, 2], ['red', 'blue']):
        raw_data = raw_df[raw_df['mpu_num'] == sensor_num]
        kalman_data = kalman_df[kalman_df['mpu_num'] == sensor_num]
        
        # Merge data by timestamp
        merged = pd.merge_asof(
            raw_data[['timestamp', 'roll_acc', 'pitch_acc']].sort_values('timestamp'),
            kalman_data[['timestamp', 'kalX_deg', 'kalY_deg']].sort_values('timestamp'),
            on='timestamp'
        )
        
        merged['roll_error'] = merged['roll_acc'] - merged['kalX_deg']
        merged['pitch_error'] = merged['pitch_acc'] - merged['kalY_deg']
        
        # Plot histograms
        axes[0, 0].hist(merged['roll_error'].dropna(), bins=50, alpha=0.7, 
                       label=f'MPU {sensor_num}', color=color)
        axes[0, 1].hist(merged['pitch_error'].dropna(), bins=50, alpha=0.7, 
                       label=f'MPU {sensor_num}', color=color)
    
    axes[0, 0].set_title('Roll Error Distribution (Raw - Kalman)')
    axes[0, 0].set_xlabel('Error (degrees)')
    axes[0, 0].set_ylabel('Frequency')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    axes[0, 1].set_title('Pitch Error Distribution (Raw - Kalman)')
    axes[0, 1].set_xlabel('Error (degrees)')
    axes[0, 1].set_ylabel('Frequency')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # Calculate sensor differences for raw and Kalman data
    # Align data by timestamp for raw data
    raw_aligned = pd.merge_asof(
        raw_mpu1[['timestamp', 'roll_acc', 'pitch_acc']].sort_values('timestamp'),
        raw_mpu2[['timestamp', 'roll_acc', 'pitch_acc']].sort_values('timestamp'),
        on='timestamp', 
        suffixes=('_mpu1', '_mpu2')
    )
    
    # Align data by timestamp for Kalman data
    kalman_aligned = pd.merge_asof(
        kalman_mpu1[['timestamp', 'kalX_deg', 'kalY_deg']].sort_values('timestamp'),
        kalman_mpu2[['timestamp', 'kalX_deg', 'kalY_deg']].sort_values('timestamp'),
        on='timestamp', 
        suffixes=('_mpu1', '_mpu2')
    )
    
    # Calculate differences
    raw_aligned['roll_diff'] = raw_aligned['roll_acc_mpu1'] - raw_aligned['roll_acc_mpu2']
    raw_aligned['pitch_diff'] = raw_aligned['pitch_acc_mpu1'] - raw_aligned['pitch_acc_mpu2']
    kalman_aligned['roll_diff'] = kalman_aligned['kalX_deg_mpu1'] - kalman_aligned['kalX_deg_mpu2']
    kalman_aligned['pitch_diff'] = kalman_aligned['kalY_deg_mpu1'] - kalman_aligned['kalY_deg_mpu2']
    
    # Plot sensor difference distributions
    axes[1, 0].hist(raw_aligned['roll_diff'].dropna(), bins=50, alpha=0.7, label='Raw', color='green')
    axes[1, 0].hist(kalman_aligned['roll_diff'].dropna(), bins=50, alpha=0.7, label='Kalman', color='orange')
    axes[1, 0].set_title('Roll Difference Distribution (MPU1 - MPU2)')
    axes[1, 0].set_xlabel('Difference (degrees)')
    axes[1, 0].set_ylabel('Frequency')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    axes[1, 1].hist(raw_aligned['pitch_diff'].dropna(), bins=50, alpha=0.7, label='Raw', color='green')
    axes[1, 1].hist(kalman_aligned['pitch_diff'].dropna(), bins=50, alpha=0.7, label='Kalman', color='orange')
    axes[1, 1].set_title('Pitch Difference Distribution (MPU1 - MPU2)')
    axes[1, 1].set_xlabel('Difference (degrees)')
    axes[1, 1].set_ylabel('Frequency')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    
    if save_plots:
        plt.savefig('error_distributions.png', dpi=300, bbox_inches='tight')
        print("Plot saved as 'error_distributions.png'")
    else:
        plt.show()

def main():
    args = parse_arguments()
    
    print("Loading data...")
    
    # Load data
    raw_df = load_and_preprocess_data(args.raw_file, args.start_time, args.end_time)
    kalman_df = load_and_preprocess_data(args.kalman_file, args.start_time, args.end_time)
    
    print(f"Raw data points: {len(raw_df)}")
    print(f"Kalman data points: {len(kalman_df)}")
    
    if len(raw_df) == 0 or len(kalman_df) == 0:
        print("No data found for the selected time range.")
        return
    
    # Create plots
    plot_raw_vs_kalman_comparison(raw_df, kalman_df, args.save_plots)
    plot_sensor_differences(raw_df, kalman_df, args.save_plots)
    plot_error_distributions(raw_df, kalman_df, args.save_plots)

if __name__ == "__main__":
    main()