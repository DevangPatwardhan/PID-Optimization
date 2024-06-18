import numpy as np
from scipy.optimize import minimize
from scipy.signal import lti, step
from scipy.optimize import NonlinearConstraint

#Add your state-space model
A = np.array([[-0.05, 1], [-0.01, -0.1]])
B = np.array([[0.1], [0.01]])  
C = np.array([[1, 0]])
D = np.array([[0]])

system = (A, B, C, D)

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

def cost_function(params):
    Kp, Ki, Kd = params
    A_cl, B_cl, C_cl, D_cl = closed_loop_state_space(Kp, Ki, Kd, system)
    sys_cl = lti(A_cl, B_cl, C_cl, D_cl)
    t, y = step(sys_cl)
    y = y.flatten()
    overshoot = np.max(y) - 1
    rise_time = np.where(y >= 1)[0][0] if np.any(y >= 1) else len(y)
    settling_time = np.where(np.abs(y - 1) < 0.02)[0][-1] if np.any(np.abs(y - 1) < 0.02) else len(y)
    cost = overshoot**2 + rise_time**2 + settling_time**2
    return cost

def stability_constraint(params):
    Kp, Ki, Kd = params[0], params[1], params[2]  # Unpack params as individual variables
    A_cl, _, _, _ = closed_loop_state_space(Kp, Ki, Kd, system)
    eigenvalues = np.linalg.eigvals(A_cl)
    return np.real(eigenvalues)

# These are for example.Add bounds and x0 according to your system. 
bounds = [(0, 10), (0, 10), (0, 10)]
nonlinear_constraint = NonlinearConstraint(stability_constraint, -np.inf, 0)
result = minimize(cost_function, x0=[1.0, 0.1, 0.1], bounds=bounds, constraints=[nonlinear_constraint], method='SLSQP')

Kp, Ki, Kd = result.x
print(f"Optimized PID gains: Kp: {Kp}, Ki: {Ki}, Kd: {Kd}")
