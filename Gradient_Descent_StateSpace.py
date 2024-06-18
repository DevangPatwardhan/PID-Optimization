import numpy as np
from scipy.signal import lti, step



# Add your linearised state-space model 
A = np.array([[-0.05, 1], [-0.01, -0.1]])
B = np.array([[0.1], [0.01]])  
C = np.array([[1, 0]])
D = np.array([[0]])

system = (A, B, C, D)

# PID parameters
Kp = 0.1
Ki = 0.1
Kd = 0.1
learning_rate = 0.0001  
max_iterations = 1000

def pid_controller(Kp, Ki, Kd):
    A_pid = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
    B_pid = np.array([[0], [0], [1]])
    C_pid = np.array([[Kp, Ki, Kd]])
    D_pid = np.array([[0]])
    return A_pid, B_pid, C_pid, D_pid

def closed_loop_state_space(Kp, Ki, Kd, system):
    A, B, C, D = system
    A_pid, B_pid, C_pid, D_pid = pid_controller(Kp, Ki, Kd)
    A_cl = np.block([[A, B @ C_pid], [B_pid @ C, A_pid]])
    B_cl = np.block([[B], [B_pid @ D]])
    C_cl = np.block([C, D @ C_pid])
    D_cl = D
    return A_cl, B_cl, C_cl, D_cl

def cost_function(Kp, Ki, Kd, system):
    A_cl, B_cl, C_cl, D_cl = closed_loop_state_space(Kp, Ki, Kd, system)
    sys_cl = lti(A_cl, B_cl, C_cl, D_cl)
    t, y = step(sys_cl)
    y = y.flatten()
    overshoot = np.max(y) - 1
    rise_time = np.where(y >= 1)[0][0] if np.any(y >= 1) else len(y)
    settling_time = np.where(np.abs(y - 1) < 0.02)[0][-1] if np.any(np.abs(y - 1) < 0.02) else len(y)
    cost = overshoot**2 + rise_time**2 + settling_time**2
    if np.isnan(cost) or np.isinf(cost):
        return 1e10  
    return cost

for i in range(max_iterations):
    cost = cost_function(Kp, Ki, Kd, system)
    
    dKp = (cost_function(Kp + 1e-5, Ki, Kd, system) - cost) / 1e-5
    dKi = (cost_function(Kp, Ki + 1e-5, Kd, system) - cost) / 1e-5
    dKd = (cost_function(Kp, Ki, Kd + 1e-5, system) - cost) / 1e-5
    
    Kp -= learning_rate * dKp
    Ki -= learning_rate * dKi
    Kd -= learning_rate * dKd
    
    Kp = np.clip(Kp, -1, 1)
    Ki = np.clip(Ki, -1, 1)
    Kd = np.clip(Kd, -1, 1)
    
    if i % 100 == 0:
        print(f"Iteration {i}, Cost: {cost}, Kp: {Kp}, Ki: {Ki}, Kd: {Kd}")

print(f"Optimized parameters: Kp: {Kp}, Ki: {Ki}, Kd: {Kd}")
