#!/usr/bin/env python3
"""Create an offline, self-contained box review page from unreviewed proposals.

The exported JSON is a review artifact, not a ready-to-train manifest. Original
images remain untouched. Each accepted frame must have all visible people boxed.
"""
import argparse
import base64
import json
from pathlib import Path

HTML = '''<!doctype html><html lang="zh"><meta charset="utf-8"><title>人物训练图审核</title>
<style>body{background:#20262d;color:#eee;font:16px sans-serif;margin:24px}button,select,input{font-size:16px;padding:8px;margin:4px}canvas{border:1px solid #8a8f96;max-width:100%;touch-action:none}#status{padding:10px}small{color:#bbb}</style>
<h2>雕塑与演员逐图审核</h2><p>红0 / 绿1 / 蓝2 / 白3 / 棕4 / 雕塑5。框住所有可见人物，从头到鞋；遮挡只标可见范围。<br>选择类别后在图中拖动新增框。点击已有框后可删除或改类。不能确认的图选择「跳过」。原图不会被修改。</p>
<div><button onclick="move(-1)">上一张</button><button onclick="move(1)">下一张</button><span id="status"></span></div>
<div><select id="cls"><option value="0">0 红演员</option><option value="1">1 绿演员</option><option value="2">2 蓝演员</option><option value="3">3 白演员</option><option value="4">4 棕演员</option><option value="5" selected>5 雕塑干扰</option></select>
<button onclick="changeClass()">选中框改类</button><button onclick="del()">删除选中框</button><button onclick="clearBoxes()">清除此图框</button></div>
<canvas id="cv" width="752" height="480"></canvas>
<div><button onclick="mark(true)">此图全部正确，接受</button><button onclick="mark(false)">跳过此图</button><input id="reviewer" placeholder="审核者名称"><button onclick="download()">保存审核 JSON</button></div>
<small>保存下载文件后交回。未点「接受」的图不会被当成已审核。可多次保存；关闭窗口前保存。</small>
<script>const rows=__ROWS__, colors=['#ff3232','#00ed70','#4a9bff','#ffffff','#b67c40','#ff58ea'];let index=0,selected=-1,start=null,img=new Image();const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
rows.forEach(r=>{r.reviewed=false;r.review_status='pending';r.boxes=structuredClone(r.proposed_boxes||[])});
function point(e){let b=cv.getBoundingClientRect();return [Math.max(0,Math.min(cv.width,(e.clientX-b.left)*cv.width/b.width)),Math.max(0,Math.min(cv.height,(e.clientY-b.top)*cv.height/b.height))]}
function draw(preview){ctx.clearRect(0,0,cv.width,cv.height);ctx.drawImage(img,0,0);rows[index].boxes.forEach((b,i)=>{let [x1,y1,x2,y2]=b.xyxy;ctx.strokeStyle=colors[b.class_id];ctx.lineWidth=i===selected?4:2;ctx.strokeRect(x1,y1,x2-x1,y2-y1);ctx.font='16px sans-serif';ctx.fillStyle=colors[b.class_id];ctx.fillText(b.class_id,x1,Math.max(16,y1-4))});if(preview){ctx.strokeStyle=colors[+document.getElementById('cls').value];ctx.strokeRect(start[0],start[1],preview[0]-start[0],preview[1]-start[1])}}
function show(){selected=-1;img.onload=()=>{cv.width=img.width;cv.height=img.height;draw()};img.src=rows[index].image_data;document.getElementById('status').textContent=`${index+1}/${rows.length} ${rows[index].image.split('/').pop()} ${rows[index].review_status}`}
function move(d){index=Math.max(0,Math.min(rows.length-1,index+d));show()}
function modified(){rows[index].reviewed=false;rows[index].review_status='pending';draw()}
function del(){if(selected>=0){rows[index].boxes.splice(selected,1);selected=-1;modified()}}
function clearBoxes(){rows[index].boxes=[];modified()}
function changeClass(){if(selected>=0){rows[index].boxes[selected].class_id=+document.getElementById('cls').value;modified()}}
function mark(ok){rows[index].reviewed=ok;rows[index].review_status=ok?'accepted':'skipped';show();if(index<rows.length-1)move(1)}
cv.onpointerdown=e=>{start=point(e);cv.setPointerCapture(e.pointerId)};cv.onpointermove=e=>{if(start)draw(point(e))};cv.onpointerup=e=>{if(!start)return;let end=point(e);if(Math.abs(end[0]-start[0])>3&&Math.abs(end[1]-start[1])>3){rows[index].boxes.push({class_id:+document.getElementById('cls').value,xyxy:[Math.min(start[0],end[0]),Math.min(start[1],end[1]),Math.max(start[0],end[0]),Math.max(start[1],end[1])]});selected=rows[index].boxes.length-1;modified()}else{selected=-1;rows[index].boxes.forEach((b,i)=>{let a=b.xyxy;if(end[0]>=a[0]&&end[0]<=a[2]&&end[1]>=a[1]&&end[1]<=a[3])selected=i})}start=null;draw()};
function download(){let who=document.getElementById('reviewer').value.trim();if(!who){alert('请填写审核者名称');return}let out=rows.map(r=>{let q={...r,reviewer:who};delete q.image_data;return q});let a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(out,null,2)],{type:'application/json'}));a.download='reviewed_people_boxes.json';a.click();URL.revokeObjectURL(a.href)}show();</script></html>'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--proposals',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();rows=json.loads(Path(args.proposals).read_text())
    for r in rows:
        from PIL import Image
        with Image.open(r['image']) as im:
            r['width'],r['height']=im.size
        suffix=Path(r['image']).suffix.lower()
        mime='image/png' if suffix=='.png' else 'image/jpeg'
        r['image_data']='data:'+mime+';base64,'+base64.b64encode(Path(r['image']).read_bytes()).decode()
    payload=json.dumps(rows,ensure_ascii=False).replace('</','<\\/')
    Path(args.output).write_text(HTML.replace('__ROWS__',payload))
    print(args.output)


if __name__=='__main__':
    main()
