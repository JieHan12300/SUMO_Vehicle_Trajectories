"""Rebuild the average traffic field and plot the recorded SUMO trajectories."""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path('data'))
    parser.add_argument('--figures', type=Path, default=Path('figures'))
    parser.add_argument('--vehicles', nargs='+', default=['veh_000487', 'veh_002058'])
    args = parser.parse_args()
    run = json.loads((args.data / 'run_summary.json').read_text(encoding='utf-8'))
    summary = pd.read_csv(args.data / 'vehicle_summary.csv')
    ids = summary.loc[summary.trajectory_samples > 0, 'vehicle_id'].sort_values().to_numpy()
    if set(args.vehicles) - set(ids):
        parser.error('Selected vehicle is absent; choose existing IDs with --vehicles.')
    display_ids = ids[np.linspace(0, len(ids)-1, min(100, len(ids)), dtype=int)]
    selected = set(display_ids) | set(args.vehicles)
    duration, length = run['demand_end_s'], run['road_length_m'] / 1609.344
    dt, dx = 300.0, 0.1
    nt, ns = round(duration / dt), round(length / dx)
    if not np.isclose(nt*dt, duration) or not np.isclose(ns*dx, length):
        raise ValueError('Road length and duration must be multiples of the traffic grid.')
    time_edges = np.arange(nt+1)*dt
    space_edges_m = np.round(np.arange(ns+1)*dx*1609.344, 6)
    pieces, displayed = [], []
    for chunk in pd.read_csv(args.data / 'trajectories.csv.gz', chunksize=500_000,
                             usecols=['vehicle_id','time_s','position_m','speed_mph']):
        displayed.append(chunk.loc[chunk.vehicle_id.isin(selected)].copy())
        # Half-open cells; records at the 6 h boundary remain in raw trajectories.
        inside = (chunk.time_s >= 0) & (chunk.time_s < duration) & (chunk.position_m >= 0) & (chunk.position_m < space_edges_m[-1])
        cell = chunk.loc[inside].copy()
        cell['space_index'] = np.searchsorted(space_edges_m, cell.position_m, side='right')-1
        cell['time_index'] = np.searchsorted(time_edges, cell.time_s, side='right')-1
        pieces.append(cell.groupby(['space_index','time_index','vehicle_id']).speed_mph.agg(['sum','count']))
    # Merge chunks before per-vehicle means, then average across vehicles.
    totals = pd.concat(pieces).groupby(level=[0,1,2]).sum()
    per_vehicle = totals['sum'] / totals['count']
    grid = per_vehicle.groupby(level=[0,1]).agg(['mean','count'])
    index = pd.MultiIndex.from_product([range(ns),range(nt)], names=['space_index','time_index'])
    grid = grid.reindex(index)
    observed = grid['mean'].notna().to_numpy().reshape(ns,nt)
    speed = grid['mean'].to_numpy().reshape(ns,nt)
    si, ti = np.indices((ns,nt))
    pd.DataFrame({
        'space_start_mile':si.ravel()*dx,'space_end_mile':(si.ravel()+1)*dx,
        'time_start_s':ti.ravel()*dt,'time_end_s':(ti.ravel()+1)*dt,
        'average_speed_mph':np.where(observed,speed,65.0).ravel(),
        'vehicle_count':grid['count'].fillna(0).astype(int).to_numpy(),
        'is_observed':observed.ravel().astype(int),
    }).to_csv(args.data / 'traffic_field.csv',index=False,float_format='%.9f')

    args.figures.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,
                         'axes.spines.right':False,'savefig.dpi':200})
    tracks = pd.concat(displayed).sort_values(['vehicle_id','time_s'])
    fig, axes = plt.subplots(2,1,figsize=(11,8),layout='constrained')
    segments, colors = [], []
    for _, vehicle in tracks.loc[tracks.vehicle_id.isin(display_ids)].groupby('vehicle_id'):
        xy = np.column_stack([vehicle.time_s.to_numpy()/3600,vehicle.position_m.to_numpy()/1609.344])
        segments.append(np.stack([xy[:-1],xy[1:]],axis=1))
        colors.append(vehicle.speed_mph.to_numpy()[:-1])
    lines=LineCollection(np.concatenate(segments),cmap='viridis',norm=plt.Normalize(20,65),linewidths=0.8)
    lines.set_array(np.concatenate(colors)); axes[0].add_collection(lines)
    axes[0].set(xlim=(0,duration/3600),ylim=(0,length),xlabel='Simulation time (h)',
                ylabel='Road position (mile)',title=f'(a) Time-space trajectories: {len(display_ids)} uniformly selected vehicles')
    fig.colorbar(lines,ax=axes[0],label='Speed (mph)',pad=0.02)
    for name in args.vehicles:
        vehicle=tracks.loc[tracks.vehicle_id==name]
        axes[1].plot(vehicle.time_s-vehicle.time_s.iloc[0],vehicle.speed_mph,
                     linewidth=1.1,label=f'No. {int(name.removeprefix("veh_"))}')
    axes[1].set(xlabel='Time since first recorded sample (s)',ylabel='Speed (mph)',
                title='(b) Individual vehicle speed profiles')
    axes[1].legend(frameon=False,ncol=len(args.vehicles))
    for ax in axes:
        ax.grid(alpha=0.18); ax.set_axisbelow(True)
    fig.savefig(args.figures / 'vehicle_trajectories.png'); plt.close(fig)

    fig,ax=plt.subplots(figsize=(11,4.5),layout='constrained')
    cmap=plt.get_cmap('viridis').copy(); cmap.set_bad('#dddddd')
    im=ax.imshow(np.ma.masked_where(~observed,speed),origin='lower',aspect='auto',
                 interpolation='nearest',extent=[0,duration/3600,0,length],cmap=cmap,vmin=20,vmax=65)
    ax.set(xlabel='Simulation time (h)',ylabel='Road position (mile)',
           title='Average traffic speed: 300 s × 0.1 mile (gray: no observations)')
    fig.colorbar(im,ax=ax,label='Speed (mph)',pad=0.02)
    fig.savefig(args.figures / 'traffic_speed_heatmap.png'); plt.close(fig)
    print(f'Saved traffic_field.csv and two figures; observed cells: {observed.sum()}/{ns*nt}.')


if __name__ == '__main__':
    main()
