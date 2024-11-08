import numpy as np
from scipy.optimize import minimize

def objective(x):
    C_h = x[0]
    C_ess = x[1]
    C_u = x[2]
    C_unet = x[3]
    #C_g = x[4]
    return C_h + C_ess + C_u + C_unet

### AC model
def indoor_temp(T_in_prev, T_out, Ci, Ri, ni, P_hvac):
    return (1-1/(Ci*Ri))*T_in_prev + 1/(Ci*Ri)*T_out - ni/Ci*P_hvac 
def con_for_T_min(x, T_in_min, T_in_prev, T_out, Ci, Ri, ni, P_hvac):
    T_in = (1-1/(Ci*Ri))*T_in_prev + 1/(Ci*Ri)*T_out - ni/Ci*P_hvac 
    return T_in - T_in_min
def con_for_T_max(x, T_in_max, T_in_prev, T_out, Ci, Ri, ni, P_hvac):
    T_in = (1-1/(Ci*Ri))*T_in_prev + 1/(Ci*Ri)*T_out - ni/Ci*P_hvac
    return T_in_max  - T_in
def constraint1(x, T_in_set, T_in_prev, T_out, Ci, Ri, ni, P_hvac, Dis_i):
    T_in = indoor_temp(T_in_prev, T_out, Ci, Ri, ni, P_hvac)
    C_h = Dis_i*(T_in- T_in_set)**2
    return  C_h - x[0]

### ESS model 
def solution_for_ESS(x): # define the cost constraint
    P_s_c, P_s_d = x[:T], x[T:]
    cost = gamma_c * np.sum(P_s_c) + gamma_d * np.sum(P_s_d)
    return cost
def soc_constraint(x): # Define the equality constraint (state of charge dynamics)
    P_s_c, P_s_d = x[:T], x[T:]
    S = np.zeros(T)
    S[0] = S0  # Initial SOC
    for t in range(1, T):
        S[t] = S[t-1] + (P_s_c[t-1] * eta_c - P_s_d[t-1] / eta_d) * delta_t
    return S
def soc_limits(x): # Define the constraint function: SOC must stay within limits
    S = soc_constraint(x)
    return np.concatenate([S - S_min, S_max - S])




### building operation cost
#def con_building(P_L, P_sell, P_)
def con_P_buy(x,P_buy, P_trans): 
    return P_trans - P_buy 
def con_P_buy_postive(x,P_buy): 
    return P_buy 
def con_P_sell_postive(x,P_sell): 
    return P_sell
def con_P_sell(x,P_sell, P_trans): 
    return P_trans - P_sell 
def constraint_utility_cost(x, r_buy, r_sell, P_buy, P_sell):
    C_u = (r_buy*P_buy + r_sell*P_sell)
    return C_u - x[2]
def constraint_network_utilization(x, r_net, d_i, P_buy, P_sell, t):
    C_unet = r_net * d_i *(P_buy -P_sell)*delta_t
    return C_unet - x[3]

x0 = np.array([0.01,1,1,1])
### variables for AC
Ci = 1.5  # Thermal capacity (kWh/C)
Ri = 1.33  # Thermal resistance (kWh/C)
T_in_prev = 20  # Previous indoor temperature
T_out = 30  # Outdoor temperature
ni = 0.15  # Efficiency factor
P_hvac = 5  # Power consumed by HVAC
T_in_min = 22  # Minimum indoor temperature (degree C)
T_in_max = 28  # Maximum indoor temperature (degree C)
T_in_set = 25  # Desired indoor temperature (degree C)
Dis_i = 2.2  # Disconfort cost ($/degree C)
P_hvac = 0.6  # (kWh)


### variables for ESS 
T = 24  # Time steps
S_min = 0  # Minimum SOC
S_max = 100  # Maximum SOC
P_s_c_max = 50  # Max charging power
P_s_d_max = 50  # Max discharging power
eta_c = 0.9  # Charging efficiency
eta_d = 0.9  # Discharging efficiency
gamma_c = 0.005  # Unit charging cost
gamma_d = 0.005  # Unit discharging cost
delta_t = 1  # Time step size (assumed 1 hour)
# Initialize variables for ESS
S0 = 50  # Initial state of charge (SOC)
S = np.zeros(T)  # State of charge array for each time step
P_s_c = np.zeros(T)  # Charging power
P_s_d = np.zeros(T)  # Discharging power
#switching_statuses = np.zeros(24)  
# Define bounds for the charging and discharging power (non-negative)
bounds = [(0, P_s_c_max)] * T + [(0, P_s_d_max)] * T

# Initial guess for the charging/discharging power
x0 = np.zeros(2 * T)


# variables for the building operation cost 
r_buy = 0.15   # Rate for buying power ($/kWh)
r_sell = 0.05  # Rate for selling power ($/kWh)
P_buy = 0    # Power bought from the grid (kW)
P_sell =  0    # Power sold to the grid (kW)
P_trans = 200  # Power capacity of the grid (kW)
r_net = 0.003  # Rate for network utilization ($/kWh)
d_i = 0.9      # Network utilization efficiency




constraint_for_T = {'type':'eq', 'fun': indoor_temp, 'args': ( T_in_prev, T_out, Ci, Ri, ni, P_hvac)}
constraint_for_T_min = {'type':'ineq', 'fun': con_for_T_min, 'args': (T_in_min, T_in_prev, T_out, Ci, Ri, ni, P_hvac)}
constraint_for_T_max = {'type':'ineq', 'fun': con_for_T_max, 'args': (T_in_max, T_in_prev, T_out, Ci, Ri, ni, P_hvac)}
con1 = {'type':'eq', 'fun': constraint1, 'args': (T_in_set, T_in_prev, T_out, Ci, Ri, ni, P_hvac, Dis_i)}



constraint_P_buy = {'type': 'ineq', 'fun': con_P_buy, 'args': (P_buy, P_trans)}
constraint_P_buy_positive = {'type': 'ineq', 'fun': con_P_buy_postive, 'args': (P_buy,)}
constraint_P_sell = {'type': 'ineq', 'fun': con_P_sell, 'args': (P_sell, P_trans)}
constraint_P_sell_positive = {'type': 'ineq', 'fun': con_P_sell_postive, 'args': (P_sell,)}
constraint_utility = {'type': 'eq', 'fun': constraint_utility_cost, 'args': (r_buy, r_sell, P_buy, P_sell)}
constraint_network = {'type': 'eq', 'fun': constraint_network_utilization, 'args': (r_net, d_i, P_buy, P_sell, delta_t)}

constraint_for_ESS = {'type': 'ineq', 'fun': soc_limits} 
cons = [constraint_for_T_min, constraint_for_T_max, con1, constraint_P_buy, constraint_P_sell, constraint_P_buy_positive, 
        constraint_P_sell_positive,constraint_utility, constraint_network]
#bounds = [(0, 100), (0, 100), (0, 100), (0, 100)]  # Modify according to your problem
sol = minimize(objective, x0=x0, method='SLSQP', bounds = bounds, constraints=cons)

#sol = minimize(objective,x0 = x0, method = 'SLSQP', constraints = cons) # which method? trust-constr or SLSQP?
print(sol)
print(f"Optimization successful: {sol.success}")
print(f"Status message: {sol.message}")
print(f"Optimal values: {sol.x}")
#P_s_c_opt, P_s_d_opt = solution_for_ESS.x[:T], solution_for_ESS.x[T:]
#S_opt = soc_constraint(solution_for_ESS.x)
