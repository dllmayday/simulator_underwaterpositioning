# 3D USBL lightweight simulation: TDOA -> LS localization -> MonteCarlo RMSE
import numpy as np
from scipy.signal import chirp
from scipy.fft import rfft, irfft
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

np.random.seed(42)

# params (lightweight for quick runs)
c = 1500.0
fs = 24000
T_sig = 0.02
t = np.arange(0, T_sig, 1/fs)
sig = chirp(t, f0=1000, f1=3000, t1=T_sig, method='linear')

array_pos = np.array([
    [0.0, 0.0, 0.0],
    [0.25, 0.0, 0.0],
    [0.0, 0.25, 0.0],
    [0.25, 0.25, 0.0]
])

# simple moving target
n_steps = 30
traj = np.zeros((n_steps, 3))
traj[:,0] = np.linspace(30.0, 40.0, n_steps)
traj[:,1] = np.linspace(5.0, -5.0, n_steps)
traj[:,2] = -10.0 + 0.3*np.sin(np.linspace(0, 2*np.pi, n_steps))

def next_pow2(n):
    return 1 << (int(np.ceil(np.log2(n))))

def apply_fractional_delay(sig, delay_sec, fs):
    n = len(sig)
    N = next_pow2(n)
    SIG = rfft(sig, n=N)
    freqs = np.fft.rfftfreq(N, d=1/fs)
    phase = np.exp(-2j * np.pi * freqs * delay_sec)
    SIG_shifted = SIG * phase
    shifted = irfft(SIG_shifted, n=N)
    return shifted[:n]

def generate_received_snapshot(sig, s_pos, array_pos, c, fs, snr_db=20):
    M = array_pos.shape[0]
    L = len(sig)
    recv = np.zeros((M, L))
    for i in range(M):
        d = np.linalg.norm(s_pos - array_pos[i])
        tau = d / c
        amp = 1.0 / (d + 1e-6)
        recv[i] += amp * apply_fractional_delay(sig, tau, fs)
    sig_power = np.mean(recv**2)
    noise_power = sig_power / (10**(snr_db/10) + 1e-12)
    noise = np.random.normal(0, np.sqrt(noise_power), recv.shape)
    return recv + noise

def gcc_phat(sig1, sig2, fs):
    n = sig1.size + sig2.size
    N = next_pow2(n)
    SIG1 = rfft(sig1, n=N)
    SIG2 = rfft(sig2, n=N)
    R = SIG1 * np.conj(SIG2)
    denom = np.abs(R)
    denom[denom < 1e-12] = 1e-12
    R /= denom
    cc = irfft(R, n=N)
    max_shift = N//2
    cc = np.concatenate((cc[-max_shift:], cc[:max_shift+1]))
    shift = np.argmax(np.abs(cc)) - max_shift
    tau = shift / float(fs)
    return tau

def tdoa_residuals(pos, array_pos, tdoas, c):
    d0 = np.linalg.norm(pos - array_pos[0])
    res = []
    for i in range(1, array_pos.shape[0]):
        di = np.linalg.norm(pos - array_pos[i])
        res.append((di - d0)/c - tdoas[i-1])
    return np.array(res)

def localize_tdoa(array_pos, tdoas, c, init_guess=None):
    if init_guess is None:
        center = array_pos.mean(axis=0)
        init_guess = center + np.array([20.0, 0.0, -8.0])
    res = least_squares(tdoa_residuals, init_guess, args=(array_pos, tdoas, c), method='lm', max_nfev=1000)
    return res.x

def run_single_simulation(traj, array_pos, c, fs, sig, snr_db=15):
    n_steps = traj.shape[0]
    est_positions = np.zeros_like(traj)
    for k in range(n_steps):
        s_true = traj[k]
        recv = generate_received_snapshot(sig, s_true, array_pos, c, fs, snr_db=snr_db)
        tdoas = []
        for i in range(1, array_pos.shape[0]):
            tau = gcc_phat(recv[0], recv[i], fs)
            tdoas.append(tau)
        pos_est = localize_tdoa(array_pos, np.array(tdoas), c)
        est_positions[k] = pos_est
    return est_positions

def monte_carlo_rmse(traj, array_pos, c, fs, sig, snr_list_db, n_trials=5):
    rmse_per_snr = []
    for snr in snr_list_db:
        sqerr_total = 0.0
        for trial in range(n_trials):
            est = run_single_simulation(traj, array_pos, c, fs, sig, snr_db=snr)
            sqerr_total += np.mean(np.sum((est - traj)**2, axis=1))
        mse = sqerr_total / n_trials
        rmse_per_snr.append(np.sqrt(mse))
    return np.array(rmse_per_snr)

snr_list_db = [0, 5, 10, 15, 20]
rmse_results = monte_carlo_rmse(traj, array_pos, c, fs, sig, snr_list_db, n_trials=5)

est_example = run_single_simulation(traj, array_pos, c, fs, sig, snr_db=12)

# plots
import matplotlib.pyplot as plt
fig = plt.figure(figsize=(9,4))
ax = fig.add_subplot(121, projection='3d')
ax.plot(traj[:,0], traj[:,1], traj[:,2], label='True')
ax.plot(est_example[:,0], est_example[:,1], est_example[:,2], label='Est')
ax.scatter(array_pos[:,0], array_pos[:,1], array_pos[:,2], s=30)
ax.set_title('Trajectory')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.legend()

ax2 = fig.add_subplot(122)
ax2.plot(np.linalg.norm(est_example - traj, axis=1))
ax2.set_title('Instant pos error (m)')
ax2.set_xlabel('time step')
plt.tight_layout()
plt.show()

plt.figure(figsize=(6,4))
plt.plot(snr_list_db, rmse_results, marker='o')
plt.title('RMSE vs SNR')
plt.xlabel('SNR (dB)'); plt.ylabel('RMSE (m)')
plt.grid(True)
plt.show()

print('SNRs:', snr_list_db)
print('RMSEs (m):', np.round(rmse_results,3))

