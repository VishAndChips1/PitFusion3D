#!/usr/bin/env python3
"""Static code analysis of the pipeline source: size, structure, complexity, declared ROS parameters, topics, threads. -> data/code_metrics.json"""
import ast, os, sys, json, re, glob
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import DATA
SRC = '/home/vish/ros2_ws/src'
FILES = sorted(set(glob.glob(f'{SRC}/landfill_camera_sim/scripts/*.py') + glob.glob(f'{SRC}/pitfusion3d_bringup/**/*.py', recursive=True)))
FILES = [f for f in FILES if '/build/' not in f and '__pycache__' not in f]
BRANCH = (ast.If, ast.For, ast.While, ast.ExceptHandler, ast.With, ast.BoolOp, ast.IfExp, ast.comprehension, ast.Assert)
def const(n):
    try: return ast.literal_eval(n)
    except Exception: return ast.unparse(n)[:60]
out = {}
for f in FILES:
    src = open(f).read(); tree = ast.parse(src); lines = src.splitlines()
    code = sum(1 for l in lines if l.strip() and not l.strip().startswith('#')); com = sum(1 for l in lines if l.strip().startswith('#')); blank = sum(1 for l in lines if not l.strip())
    info = dict(path=f.replace(SRC + '/', ''), lines=len(lines), code=code, comment=com, blank=blank, doc=(ast.get_docstring(tree) or '').strip().split('\n\n')[0][:600], classes=[], functions=[], params=[], topics=dict(pub=[], sub=[]), threads=[], imports=sorted({(n.module or '').split('.')[0] if isinstance(n, ast.ImportFrom) else a.name.split('.')[0] for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom)) for a in (n.names if isinstance(n, ast.Import) else [None])}), qos_uses=len(re.findall('qos_profile_sensor_data|QoSProfile', src)))
    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        for m in cls.body:
            if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef)): m.parent_class = cls.name
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef): info['classes'].append(dict(name=node.name, line=node.lineno, methods=[m.name for m in node.body if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef))], doc=(ast.get_docstring(node) or '').split('\n')[0][:160]))
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            cc = 1 + sum(isinstance(x, BRANCH) for x in ast.walk(node)); info['functions'].append(dict(name=node.name, line=node.lineno, length=(node.end_lineno - node.lineno + 1), args=[a.arg for a in node.args.args], complexity=cc, doc=(ast.get_docstring(node) or '').split('\n')[0][:140], doc_full=' '.join((ast.get_docstring(node) or '').split())[:420], parent=getattr(node, 'parent_class', None)))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            nm = node.func.attr
            if nm == 'declare_parameter' and node.args: info['params'].append(dict(name=const(node.args[0]), default=const(node.args[1]) if len(node.args) > 1 else None, line=node.lineno))
            if nm in ('create_publisher',) and len(node.args) >= 2: info['topics']['pub'].append(dict(topic=const(node.args[1]), type=ast.unparse(node.args[0])[:40], line=node.lineno))
            if nm in ('create_subscription',) and len(node.args) >= 2: info['topics']['sub'].append(dict(topic=const(node.args[1]), type=ast.unparse(node.args[0])[:40], line=node.lineno))
            if nm == 'Subscriber' and len(node.args) >= 3: info['topics']['sub'].append(dict(topic=const(node.args[2]), type=ast.unparse(node.args[1])[:40], line=node.lineno, via='message_filters'))
        if isinstance(node, ast.Call) and ((isinstance(node.func, ast.Attribute) and node.func.attr in ('Thread', 'ThreadingTCPServer', 'Process', 'Popen', 'ThreadPoolExecutor')) or (isinstance(node.func, ast.Name) and node.func.id in ('Thread', 'Popen'))):
            info['threads'].append(dict(kind=ast.unparse(node.func)[:40], line=node.lineno, target=next((ast.unparse(k.value)[:40] for k in node.keywords if k.arg == 'target'), None)))
    info['max_complexity'] = max([x['complexity'] for x in info['functions']] or [0]); info['mean_complexity'] = round(sum(x['complexity'] for x in info['functions']) / max(len(info['functions']), 1), 2)
    out[info['path']] = info
tot = dict(files=len(out), lines=sum(v['lines'] for v in out.values()), code=sum(v['code'] for v in out.values()), comment=sum(v['comment'] for v in out.values()), blank=sum(v['blank'] for v in out.values()), functions=sum(len(v['functions']) for v in out.values()), classes=sum(len(v['classes']) for v in out.values()), params=sum(len(v['params']) for v in out.values()))
other = {}
for pat in ('**/*.yaml', '**/*.xml', '**/*.json', '**/*.sh', '**/*.html', '**/*.md', '**/*.cfg', '**/*.txt'):
    fs = [f for f in glob.glob(f'{SRC}/**/{pat.split("/")[-1]}', recursive=True) if 'livox_ros2_avia/livox_ros2_avia/src' not in f and '/build/' not in f and '/install/' not in f]
    other[pat.split('.')[-1]] = dict(files=len(fs), lines=sum(len(open(f, errors='ignore').read().splitlines()) for f in fs if os.path.isfile(f)))
cpp = [f for ext in ('cpp', 'h', 'hpp', 'cc', 'c') for f in glob.glob(f'{SRC}/**/*.{ext}', recursive=True) if '/build/' not in f]
other['cpp_vendor_driver'] = dict(files=len(cpp), lines=sum(len(open(f, errors='ignore').read().splitlines()) for f in cpp))
json.dump(dict(totals=tot, other=other, files=out), open(f'{DATA}/code_metrics.json', 'w'), indent=1)
print(json.dumps(tot), json.dumps(other))
for p, v in sorted(out.items(), key=lambda kv: -kv[1]['lines'])[:30]: print(f"{p:75s} {v['lines']:5d} lines {len(v['functions']):3d} fn  maxCC {v['max_complexity']:3d}  params {len(v['params'])}")
