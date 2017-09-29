import pylab as pl
import pandas as pd

suffixes = [
        'activation',
        'norm_fiber_length',
        'norm_fiber_velocity',
        #'norm_tendon_force',
        'other_controls',
        'tendon_force'
        ]

pl.ion()

for suffix in suffixes:
    df = pd.read_csv('testGait10dof18musc_GSO_solution_%s.sto' % suffix,
            skiprows=3, index_col=0, delimiter='\t')
    std = pd.read_csv('std_testGait10dof18musc_GSO_solution_%s.sto' % suffix,
            skiprows=3, index_col=0, delimiter='\t')
    fig =pl.figure(figsize=(4 * len(std.columns), 4))
    fig.suptitle(suffix)
    for i, col in enumerate(std.columns):
        ax = fig.add_subplot(1, len(std.columns), i + 1)
        pl.plot(std.index, std[col], label='std')
        pl.plot(df.index, df[col], label='actual')
        if i == 0:
            pl.legend()
#    df.plot(subplots=True)
#    std.plot(subplots=True)
