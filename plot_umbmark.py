import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('umbmark.csv')
ccw = df[df['direction'] == 'ccw']
cw = df[df['direction'] == 'cw']

cg_x_ccw, cg_y_ccw = ccw['eps_x'].mean(), ccw['eps_y'].mean()
cg_x_cw, cg_y_cw = cw['eps_x'].mean(), cw['eps_y'].mean()

plt.figure(figsize=(8, 8))
plt.scatter(ccw['eps_x'], ccw['eps_y'], color='blue', alpha=0.7, label='CCW Runs')
plt.scatter(cw['eps_x'], cw['eps_y'], color='red', alpha=0.7, label='CW Runs')

plt.plot(cg_x_ccw, cg_y_ccw, 'bX', markersize=12, label=f'CG CCW ({cg_x_ccw:.3f}, {cg_y_ccw:.3f})')
plt.plot(cg_x_cw, cg_y_cw, 'rX', markersize=12, label=f'CG CW ({cg_x_cw:.3f}, {cg_y_cw:.3f})')

plt.axhline(0, color='black', linestyle=':', alpha=0.6)
plt.axvline(0, color='black', linestyle=':', alpha=0.6)
plt.title('UMBmark Calibration End Points and Centers of Gravity', fontsize=13, fontweight='bold')
plt.xlabel('$\epsilon_x$ (m)', fontsize=11)
plt.ylabel('$\epsilon_y$ (m)', fontsize=11)
plt.gca().set_aspect('equal', adjustable='box')
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend()
plt.tight_layout()
plt.savefig('umbmark_plot.png', dpi=300)
plt.show()