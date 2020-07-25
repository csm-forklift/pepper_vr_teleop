'''
Generates the jacobian function for the optimization objective.
'''

import sympy as sp

# Allowes pretty printing in the terminal. Use: sp.pprint(<object>)
sp.init_printing(use_unicode=True)

#--- Constants ---#
# Pepper setpoint position
spx, spy, spz = sp.symbols('spx spy spz')
Psp = sp.Matrix([spx, spy, spz])

# Controller X axis vector
rxx, rxy, rxz = sp.symbols('rxx rxy rxz')
Rxc = sp.Matrix([rxx, rxy, rxz])

#--- Variables ---#
# (sp: shoulder pitch, sr: shoulder roll, ey: elbow yaw, er: elbow roll)
qsp, qsr, qey, qer = sp.symbols('qsp qsr qey qer') # joint angles
q = sp.Matrix([qsp, qsr, qey, qer])


#--- Transformation Matrix ---#
# Euler rotation matrices
def Rx(theta):
    return sp.Matrix([[1, 0, 0],
                      [0, sp.cos(theta), -sp.sin(theta)],
                      [0, sp.sin(theta), sp.cos(theta)]])

def Ry(theta):
    return sp.Matrix([[sp.cos(theta), 0, sp.sin(theta)],
                      [0, 1, 0],
                      [-sp.sin(theta), 0, sp.cos(theta)]])

def Rz(theta):
    return sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0],
                      [sp.sin(theta), sp.cos(theta), 0],
                      [0, 0, 1]])

# Translations
# Left Arm
d_bsl = [-0.057, 0.14974, 0.08682]
d_sel = [0.1812, 0.015, 0.00013]
d_ewl = [0.1517, 0.0, 0.0]
d_whl = [0.0695, 0.0, -0.03030]

# Right Arm
d_bsr = [-0.057, -0.14974, 0.08682]
d_ser = [0.1812, -0.015, 0.00013]
d_ewr = [0.1517, 0.0, 0.0]
d_whr = [0.0695, 0.0, -0.03030]

# Symbolic
bsx, bsy, bsz = sp.symbols('bsx bsy bsz')
sex, sey, sez = sp.symbols('sex sey sez')
ewx, ewy, ewz = sp.symbols('ewx ewy ewz')
whx, why, whz = sp.symbols('whx why whz')
d_bs = sp.Matrix([bsx, bsy, bsz])
d_se = sp.Matrix([sex, sey, sez])
d_ew = sp.Matrix([ewx, ewy, ewz])
d_wh = sp.Matrix([whx, why, whz])

# Transforms
b_T_s = sp.zeros(4,4)
b_T_s[0:3,0:3] = Ry(qsp)*Rz(qsr)
b_T_s[0:3,3] = d_bs
b_T_s[3,3] = 1.0

s_T_e = sp.zeros(4,4)
s_T_e[0:3,0:3] = Rx(qey)*Rz(qer)
s_T_e[0:3,3] = d_se
s_T_e[3,3] = 1.0

e_T_w = sp.zeros(4,4)
e_T_w[0:3,0:3] = sp.eye(3)
e_T_w[0:3,3] = d_ew
e_T_w[3,3] = 1.0

w_T_h = sp.zeros(4,4)
w_T_h[0:3,0:3] = sp.eye(3)
w_T_h[0:3,3] = d_wh
w_T_h[3,3] = 1.0

b_T_h = b_T_s * s_T_e * e_T_w * w_T_h

#--- Cost Function ---#
# Pepper's hand position
Pp = b_T_h[0:3,3]

# Pepper's hand X axis vector
Rxp = b_T_h[0:3,0]

# Position Error
Pe = Psp - Pp

# Error weights
wp = sp.symbols('wp') # position
wo = sp.symbols('wo') # orientation

# Total Cost
ct = (wp*(Pe.T*Pe))[0] + wo*(sp.acos(Rxp.T*Rxc)/sp.pi)

sp.pprint(ct)
