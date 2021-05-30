from c172p_model import *
import matplotlib.pyplot as plt

J0      = 0
alpha0  = 0
deltaf0 = 0

J      = np.linspace(-0.1,5.1,1000)
#J      = CT[:,0]
alpha  = np.linspace(-0.1,1.6,1000)
deltaf = np.linspace(-1,31,1000)

plt.plot(J,CT_interp(J),'.-',J,parder_J_CT_interp(J),'.')
plt.show()

plt.plot(deltaf,CD2_interp(deltaf),'.-',deltaf,parder_deltaf_CD2_interp(deltaf),'.')
plt.show()

plt.plot(alpha,CD3_interp(alpha,deltaf0),'.-',alpha,parder_alpha_CD3_interp(alpha,deltaf0),'.')
plt.show()

plt.plot(deltaf,CD3_interp(alpha0,deltaf),'.-',deltaf,parder_deltaf_CD3_interp(alpha0,deltaf),'.')
plt.show()

plt.plot(alpha,CD3_interp(alpha,deltaf0),'.-',alpha,parder_alpha_CD3_interp(alpha,deltaf0),'.')
plt.show()

plt.plot(deltaf,CD3_interp(alpha0,deltaf),'.-',deltaf,parder_alpha_CD3_interp(alpha0,deltaf),'.')
plt.show()
