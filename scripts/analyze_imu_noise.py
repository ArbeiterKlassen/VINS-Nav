#!/usr/bin/env python3
"""Analyze IMU noise from ROS bag: Allan variance + PSD estimation."""
import sys
import numpy as np
import rosbag
from collections import defaultdict

def allan_variance(data, dt, max_clusters=100):
    """Compute Allan variance for various tau values.
    Returns (taus, allan_dev) arrays."""
    N = len(data)
    taus = []
    allan_devs = []

    # Cluster sizes: powers of 2 up to N/2
    m_max = min(max_clusters, N // 2)
    for m in range(1, m_max + 1):
        tau = m * dt
        # Number of full clusters
        n_clusters = N // m
        if n_clusters < 2:
            break
        # Average each cluster
        clusters = np.array([np.mean(data[i*m:(i+1)*m]) for i in range(n_clusters)])
        # Allan variance: 1/(2*(M-1)) * sum((c_{k+1} - c_k)^2)
        avar = 0.5 * np.mean(np.diff(clusters)**2)
        taus.append(tau)
        allan_devs.append(np.sqrt(avar))

    return np.array(taus), np.array(allan_devs)

def fit_allan_white_noise(taus, allan_dev):
    """Fit white noise coefficient N from Allan deviation.
    Model: sigma(tau) = N / sqrt(tau)
    Fit at the smallest tau values where white noise dominates."""
    # Use first 20% of tau values (where white noise dominates)
    n = max(3, len(taus) // 5)
    log_tau = np.log(taus[:n])
    log_dev = np.log(allan_dev[:n])
    slope, intercept = np.polyfit(log_tau, log_dev, 1)
    N = np.exp(intercept + 0.5 * log_tau[0] - slope * log_tau[0])
    # More precise: sigma(1) = N if sigma(tau) = N/sqrt(tau)
    N = allan_dev[0] * np.sqrt(taus[0])
    return N

def fit_allan_random_walk(taus, allan_dev):
    """Fit bias random walk K from Allan deviation.
    Model: sigma(tau) = K * sqrt(tau/3)
    Fit at larger tau values where random walk dominates."""
    # Use last 30% of tau values
    n = len(taus) // 3
    if n < 3:
        n = len(taus)
    log_tau = np.log(taus[-n:])
    log_dev = np.log(allan_dev[-n:])
    slope, intercept = np.polyfit(log_tau, log_dev, 1)
    # sigma(tau) = K * sqrt(tau/3) -> K = sigma * sqrt(3/tau)
    K = allan_dev[-1] * np.sqrt(3.0 / taus[-1])
    return K

def analyze_bag(bagfile):
    bag = rosbag.Bag(bagfile)

    accel_data = defaultdict(list)  # axis -> values
    gyro_data = defaultdict(list)
    timestamps = []

    for topic, msg, t in bag.read_messages(topics=['/imu']):
        ts = msg.header.stamp.to_sec()
        timestamps.append(ts)
        accel_data['x'].append(msg.linear_acceleration.x)
        accel_data['y'].append(msg.linear_acceleration.y)
        accel_data['z'].append(msg.linear_acceleration.z)
        gyro_data['x'].append(msg.angular_velocity.x)
        gyro_data['y'].append(msg.angular_velocity.y)
        gyro_data['z'].append(msg.angular_velocity.z)

    bag.close()

    # Convert to numpy
    for k in accel_data:
        accel_data[k] = np.array(accel_data[k])
        gyro_data[k] = np.array(gyro_data[k])
    timestamps = np.array(timestamps)

    dt = np.median(np.diff(timestamps))
    print(f'IMU samples: {len(timestamps)}, dt={dt:.6f}s, rate={1.0/dt:.1f}Hz')
    print()

    for name, data_dict, unit in [
        ('Accelerometer', accel_data, 'm/s^2'),
        ('Gyroscope', gyro_data, 'rad/s')
    ]:
        print(f'=== {name} ===')
        for axis in ['x', 'y', 'z']:
            d = data_dict[axis]
            mean = np.mean(d)
            std = np.std(d)

            # Simple white noise estimate
            # noise_density = sigma_discrete * sqrt(dt)
            noise_density = std * np.sqrt(dt)

            # Compute Allan variance
            taus, ad = allan_variance(d, dt, max_clusters=min(300, len(d)//2))

            # Fit noise parameters
            N = fit_allan_white_noise(taus, ad)
            K = fit_allan_random_walk(taus, ad)

            print(f'  Axis {axis}: mean={mean:.6f}, std={std:.6f} {unit}')
            print(f'    Simple white noise density: {noise_density:.6f} {unit}/sqrt(Hz)')
            print(f'    Allan white noise N:       {N:.6f} {unit}/sqrt(Hz)')
            print(f'    Allan random walk K:       {K:.6f}')

        # Summary: use the max across axes (conservative)
        noise_vals = [np.std(data_dict[a]) * np.sqrt(dt) for a in ['x', 'y', 'z']]
        print(f'  Recommended noise_density:    {max(noise_vals):.6f} {unit}/sqrt(Hz)')
        print()

if __name__ == '__main__':
    analyze_bag(sys.argv[1] if len(sys.argv) > 1 else '/tmp/imu_data.bag')
