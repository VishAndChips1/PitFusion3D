import json, csv, os, sys, numpy as np
H=os.path.dirname(os.path.abspath(__file__)); OUT=sys.argv[1] if len(sys.argv)>1 else H
def _l(tag):
    p=f'{H}/results_{tag}.jsonl'; return [json.loads(l) for l in open(p)] if os.path.exists(p) else []
def load(tag):
    R=_l(tag)+(_l('merged2') if tag=='merged' else [])
    d={r['cap']:r for r in R}; return [d[k] for k in sorted(d)]
def f(x,d=1): return '-' if x is None else f'{x:.{d}f}'
def table(rows,view):
    hdr="| cap | points/cloud | KB/cloud | clouds/s | viewer Mbit/s | gap p95 ms | resp p95 ms | NN spacing cm | 2cm cover % | 5cm cover % | fused Hz | merged Hz | pipeline CPU % | streamer CPU % | system CPU % | core A p99 | core A max | est input A max (cons.) | EXT5V min | SoC max C | real clients |"
    L=[hdr,'|'+'---|'*hdr.count('|')[:0]] if False else [hdr,'|'+'---|'*(hdr.count('|')-1)]
    for r in rows:
        q=r.get('quality') or {}; p=r['power']; c=r['cpu']; t=r['topics']
        L.append(f"| {r['cap']:,} | {f(r['points_per_cloud'],0)} | {f(r['wire_kb_mean'],0)} | {f(r['updates_per_s'],2)} | {f(r['viewer_mbit_s'],2)} | {f(r['gap_ms']['p95'],0)} | {f(r['resp_ms']['p95'],0)} | {f(q.get('nn_median_cm'),2)} | {f(q.get('coverage_2cm_pct'))} | {f(q.get('coverage_5cm_pct'))} | {f(t.get('points_rgbt',{}).get('hz'),2)} | {f(t.get('points_merged',{}).get('hz'),2)} | {f(c['pipeline_pct'],0)} | {f(c['streamer_pct'],0)} | {f(c['system_pct_of_400'],0)} | {f(p['core_a']['p99'],2)} | {f(p['core_a']['max'],2)} | {f(p['est_input_a']['max'],2)} ({f(p['est_input_conservative_a_max'],2)}) | {f(p['ext5v_min'],3)} | {f(p['temp_c_max'],1)} | {r['real_clients']['min']}-{r['real_clients']['max']} |")
    return '\n'.join(L)
def plots():
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig,ax=plt.subplots(2,3,figsize=(15,8))
    for tag,col in (('merged','#4A5D7A'),('live','#B5532A')):
        R=load(tag)
        if not R: continue
        x=[r['cap'] for r in R]; g=lambda fn:[fn(r) for r in R]
        ax[0,0].plot(x,g(lambda r:(r.get('quality') or {}).get('coverage_5cm_pct')),'o-',c=col,label=f'{tag} 5cm'); ax[0,0].plot(x,g(lambda r:(r.get('quality') or {}).get('coverage_2cm_pct')),'s--',c=col,alpha=.6,label=f'{tag} 2cm')
        ax[0,1].plot(x,g(lambda r:r['viewer_mbit_s']),'o-',c=col,label=tag)
        ax[0,2].plot(x,g(lambda r:r['updates_per_s']),'o-',c=col,label=f'{tag} clouds/s'); ax[0,2].plot(x,g(lambda r:r['topics'].get('points_rgbt',{}).get('hz')),'x:',c=col,alpha=.6,label=f'{tag} fused Hz')
        ax[1,0].plot(x,g(lambda r:r['cpu']['pipeline_pct']),'o-',c=col,label=f'{tag} pipeline'); ax[1,0].plot(x,g(lambda r:r['cpu']['streamer_pct']),'s--',c=col,alpha=.6,label=f'{tag} streamer')
        ax[1,1].plot(x,g(lambda r:r['power']['est_input_a']['max']),'o-',c=col,label=f'{tag} est. input A max'); ax[1,1].plot(x,g(lambda r:r['power']['est_input_conservative_a_max']),'s--',c=col,alpha=.6,label=f'{tag} conservative')
        ax[1,2].plot(x,g(lambda r:r['power']['core_a']['p99']),'o-',c=col,label=f'{tag} VDD_CORE p99'); ax[1,2].plot(x,g(lambda r:r['power']['core_a']['max']),'s--',c=col,alpha=.6,label=f'{tag} VDD_CORE max')
    for a,t,y in zip(ax.flat,['Fidelity vs full cloud','Bandwidth to one viewer','Refresh rate','CPU (% of one core)','Estimated input current','VDD_CORE rail current'],['% of full-cloud voxels covered','Mbit/s','Hz','%','A','A']):
        a.set_title(t); a.set_xlabel('point cap'); a.set_ylabel(y); a.grid(alpha=.3); a.legend(fontsize=7)
    ax[1,1].axhline(4.5,c='r',ls=':'); ax[1,1].axhline(5.0,c='r'); ax[1,2].axhline(5.0,c='gray',ls=':')
    fig.tight_layout(); fig.savefig(f'{OUT}/pointcap_sweep.png',dpi=150)
if __name__=='__main__':
    for tag in ('merged','live'):
        R=load(tag); 
        if R: open(f'{OUT}/table_{tag}.md','w').write(table(R,tag)+'\n'); print(tag,len(R),'steps')
    plots(); print('plots written')
