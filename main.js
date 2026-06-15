"use strict";
(function(){
/* ============================ Constants ============================ */
const RHO=1.225, MU=1.81e-5, ASOUND=343;
const DEG=Math.PI/180, ASTALL=16;
const C_MPH=2.236936, C_LBF=0.2248089, C_PSI=0.000145038, C_IN=0.03937008;

/* ============================ State ============================ */
const state={
  speed:60, size:75, aoa:0, obj:'sphere', modelName:'sphere',
  paused:false, zoomIdx:0, gtab:'cp', units:'SI', stallWarnings:true,
  overlays:{streamlines:true,pressure:false,forces:true,boundary:false,labels:false,grid:false}
};
const ZOOMS=[1,1.4,2,3];
let undoStack=[], redoStack=[];

/* STL data */
let stlTris=null, stlBounds=null, stlContour=null;
let stlHalfW=1, stlHalfH=1;

/* 3D pressure view */
let three3d_scene=null,three3d_cam=null,three3d_ren=null,three3d_mesh=null,raf3d=0;
let freestreamBase3d=[1,0,0], cpScaleFactor=1.0, arrow3d=null;
const orb={theta:0.6,phi:1.1,r:3.5,down:false,ox:0,oy:0,ot:0,op:0};

/* ============================ Canvas setup ============================ */
const tunnel=document.getElementById('tunnel');
const tctx=tunnel.getContext('2d');
const graph=document.getElementById('graph');
const gctx=graph.getContext('2d');
const dpr=window.devicePixelRatio||1;
let TW=0,TH=0;
function resize(){
  const w=tunnel.clientWidth, h=tunnel.clientHeight; TW=w; TH=h;
  tunnel.width=Math.round(w*dpr); tunnel.height=Math.round(h*dpr);
  tctx.setTransform(dpr,0,0,dpr,0,0);
  graph.width=260*dpr; graph.height=140*dpr; gctx.setTransform(dpr,0,0,dpr,0,0);
  if(typeof LB!=='undefined') LB.inited=false;   // rebuild the lattice for the new size
}
window.addEventListener('resize',resize);

/* ============================ Units ============================ */
function uSpeed(v){ return state.units==='SI'? v : v*C_MPH; }
function uForce(n){ return state.units==='SI'? n : n*C_LBF; }
function uPress(p){ return state.units==='SI'? p : p*C_PSI; }
function uLen(mm){ return state.units==='SI'? mm : mm*C_IN; }
function lblSpeed(){ return state.units==='SI'?'m/s':'mph'; }
function lblForce(){ return state.units==='SI'?'N':'lbf'; }
function lblPress(){ return state.units==='SI'?'Pa':'psi'; }
function lblLen(){ return state.units==='SI'?'mm':'in'; }

/* ============================ Coefficients (genuine full-range stall) ============================ */
function getCoeffs(obj, aoaDeg, Re, Mach){
  if(obj==='sphere'){
    let Cd=(Re>3e5)?0.20:0.47;
    if(Mach>0.8) Cd += 0.6*Math.min(1,(Mach-0.8)/0.5);
    return {Cd,Cl:0,stall:false};
  }
  // Lifting/slender body, valid across the whole -90..+90 range.
  const a=aoaDeg*DEG, aa=Math.abs(aoaDeg);
  const tau=Math.max(0.05,Math.min(1, stlHalfH/Math.max(stlHalfW,1e-3)));
  const Cd0=0.05+0.9*tau*tau;
  // (1) attached-flow branch
  const slope=2*Math.PI*0.7;
  const clAtt=slope*Math.sin(a);
  const cdAtt=Cd0 + clAtt*clAtt*0.06 + 0.012*Math.max(0,aa-12);
  // (2) fully separated flat-plate branch (Hoerner/Viterna), good to +-90
  const CdMax=2.0;
  const clSt=0.5*CdMax*Math.sin(2*a);
  const cdSt=Cd0 + CdMax*Math.sin(a)*Math.sin(a);
  // (3) smooth blend across the stall angle
  const w=Math.max(0,Math.min(1,(aa-ASTALL)/8)), sm=w*w*(3-2*w);
  let Cl=(1-sm)*clAtt + sm*clSt;
  let Cd=(1-sm)*cdAtt + sm*cdSt;
  if(Mach<0.95){ const pg=Math.sqrt(1-Mach*Mach); Cd/=pg; Cl/=pg; }
  else { Cd += 0.15 + 0.25*Math.min(1,(Mach-0.95)/0.55); }
  return {Cd,Cl,stall:aa>ASTALL};
}

/* ============================ Physics ============================ */
const phys={V:60,Re:0,Mach:0,Cd:0,Cl:0,Fd:0,Fl:0,q:0,A:0,LD:0,D:0,stall:false};
function computePhysics(){
  const V=state.speed, D=state.size/1000;
  const Re=RHO*V*D/MU, Mach=V/ASOUND;
  // Cd / Cl are MEASURED from the live LBM flow (boundary momentum exchange),
  // calibrated to a sane scale. Re/Mach/q come from the real air speed.
  let Cd=Math.abs(measCd)*CAL, Cl=measCl*CAL;
  if(Mach>0.8) Cd*=(1+0.6*Math.min(1,(Mach-0.8)/0.5));   // compressibility drag rise
  const stall = state.obj==='stl' && Math.abs(state.aoa)>16;
  const q=0.5*RHO*V*V, A=Math.PI*Math.pow(D/2,2);
  const Fd=q*A*Cd, Fl=q*A*Cl, LD=Cd>1e-6?Cl/Cd:0;
  Object.assign(phys,{V,Re,Mach,Cd,Cl,Fd,Fl,q,A,LD,D,stall});
}

/* ============================ Body geometry ============================ */
let cx=0,cy=0,Rpx=60,Upx=2;
function bodyParams(){
  cx=TW*0.40; cy=TH*0.5;
  Rpx=(state.size/200)*Math.min(TW,TH)*0.28+14;
  Rpx*=ZOOMS[state.zoomIdx];
  if(state.obj!=='stl') Rpx=Math.min(Rpx,TH*0.44);
  Upx=(state.speed/60)*2.2;
}
function getSilhouette(){
  const a=state.aoa*DEG, ca=Math.cos(a), sa=Math.sin(a); const pts=[];
  if(state.obj==='stl' && stlContour && stlContour.length>3){
    for(const p of stlContour){ const x=p.x*Rpx, y=p.y*Rpx; pts.push({x:cx+x*ca-y*sa, y:cy+x*sa+y*ca}); }
  } else {
    for(let i=0;i<64;i++){ const th=i/64*Math.PI*2; pts.push({x:cx+Math.cos(th)*Rpx, y:cy+Math.sin(th)*Rpx}); }
  }
  return pts;
}

/* ============================ Lattice-Boltzmann fluid solver (D2Q9) ============================ */
/* A real CFD solver: the body is rasterised into solid no-slip cells, so flow
   genuinely goes around it (impermeable), forms a real wake, and the forces are
   read out from momentum exchange on the boundary. */
const EX=[0,1,0,-1,0,1,-1,-1,1], EY=[0,0,1,0,-1,1,1,-1,-1];
const WT=[4/9,1/9,1/9,1/9,1/9,1/36,1/36,1/36,1/36], OPP=[0,3,4,1,2,7,8,5,6];
const LB_SUBSTEPS=3, CAL=0.14;          // substeps/frame, force calibration to a sane Cd scale
const LB={NX:0,NY:0,CS:1,f:null,f2:null,ux:null,uy:null,rho:null,solid:null,fin:null,
  uin:0.1,tau:0.6,Fx:0,Fy:0,Hfront:2,bxL:0,bxR:0,byT:0,byB:0,inited:false,geomKey:''};
let measCd=0, measCl=0;
function lbFeq(out,rho,ux,uy){ const usq=ux*ux+uy*uy;
  for(let k=0;k<9;k++){ const eu=EX[k]*ux+EY[k]*uy; out[k]=WT[k]*rho*(1+3*eu+4.5*eu*eu-1.5*usq); } }
function lbInit(){
  const NX=Math.max(100,Math.min(280,Math.round(TW/3.2)));
  const NY=Math.max(40,Math.round(NX*TH/Math.max(TW,1)));
  LB.NX=NX; LB.NY=NY; LB.CS=TW/NX; const n=NX*NY;
  LB.f=new Float32Array(n*9); LB.f2=new Float32Array(n*9);
  LB.ux=new Float32Array(n); LB.uy=new Float32Array(n); LB.rho=new Float32Array(n);
  LB.solid=new Uint8Array(n); LB.fin=new Float32Array(9); lbFeq(LB.fin,1,LB.uin,0);
  for(let c=0;c<n;c++){ for(let k=0;k<9;k++) LB.f[c*9+k]=LB.fin[k]; LB.rho[c]=1; LB.ux[c]=LB.uin; }
  measCd=0; measCl=0; LB.inited=true; LB.geomKey='';
}
function pointInPolyXY(x,y,poly){ let inside=false;
  for(let i=0,j=poly.length-1;i<poly.length;j=i++){ const xi=poly[i].x,yi=poly[i].y,xj=poly[j].x,yj=poly[j].y;
    if(((yi>y)!==(yj>y)) && (x<(xj-xi)*(y-yi)/(yj-yi)+xi)) inside=!inside; } return inside; }
function lbBuildMask(){
  const NX=LB.NX,NY=LB.NY,CS=LB.CS, pts=getSilhouette();
  let mnx=1e9,mny=1e9,mxx=-1e9,mxy=-1e9;
  for(const p of pts){mnx=Math.min(mnx,p.x);mny=Math.min(mny,p.y);mxx=Math.max(mxx,p.x);mxy=Math.max(mxy,p.y);}
  LB.solid.fill(0); LB.bxL=NX;LB.bxR=0;LB.byT=NY;LB.byB=0; let any=false;
  for(let j=0;j<NY;j++){ const y=(j+0.5)*CS; if(y<mny-CS||y>mxy+CS) continue;
    for(let i=0;i<NX;i++){ const x=(i+0.5)*CS; if(x<mnx-CS||x>mxx+CS) continue;
      if(pointInPolyXY(x,y,pts)){ LB.solid[j*NX+i]=1; any=true;
        if(i<LB.bxL)LB.bxL=i; if(i>LB.bxR)LB.bxR=i; if(j<LB.byT)LB.byT=j; if(j>LB.byB)LB.byB=j; } } }
  if(!any){ LB.bxL=LB.bxR=NX>>1; LB.byT=LB.byB=NY>>1; }
  LB.Hfront=Math.max(2,LB.byB-LB.byT+1);
}
function lbUpdateTau(){
  const reL=120+780*Math.min(1, Math.log10(1+phys.Re)/7);  // higher lattice Re -> thinner BL, flow hugs the shape
  const nu=LB.uin*LB.Hfront/Math.max(reL,1);
  LB.tau=Math.max(0.508, Math.min(1.2, 3*nu+0.5));
}
function lbStep(){
  const NX=LB.NX,NY=LB.NY,f=LB.f,f2=LB.f2,solid=LB.solid,fin=LB.fin,omega=1/LB.tau;
  for(let j=0;j<NY;j++)for(let i=0;i<NX;i++){ const c=j*NX+i; if(solid[c])continue; const b=c*9;
    let rho=0,ux=0,uy=0;
    for(let k=0;k<9;k++){ const fk=f[b+k]; rho+=fk; ux+=fk*EX[k]; uy+=fk*EY[k]; }
    if(rho<1e-6) rho=1e-6; ux/=rho; uy/=rho;
    LB.rho[c]=rho; LB.ux[c]=ux; LB.uy[c]=uy;
    const usq=ux*ux+uy*uy;
    for(let k=0;k<9;k++){ const eu=EX[k]*ux+EY[k]*uy; const fe=WT[k]*rho*(1+3*eu+4.5*eu*eu-1.5*usq); f[b+k]+=omega*(fe-f[b+k]); }
  }
  for(let j=0;j<NY;j++)for(let i=0;i<NX;i++){ const c=j*NX+i; if(solid[c])continue; const b=c*9;
    for(let k=0;k<9;k++){ const si=i-EX[k], sj=j-EY[k]; let val;
      if(sj<0||sj>=NY||si<0) val=fin[k];                 // inlet + top/bottom freestream
      else if(si>=NX) val=f[b+k];                        // outlet (zero gradient)
      else { const sc=sj*NX+si; if(solid[sc]) val=f[b+OPP[k]]; else val=f[sc*9+k]; } // bounce-back / stream
      f2[b+k]=val; } }
  const t=LB.f; LB.f=f2; LB.f2=t;
}
function lbForces(){
  const NX=LB.NX,NY=LB.NY,f=LB.f,solid=LB.solid; let Fx=0,Fy=0;
  for(let j=Math.max(0,LB.byT-1);j<=Math.min(NY-1,LB.byB+1);j++)
    for(let i=Math.max(0,LB.bxL-1);i<=Math.min(NX-1,LB.bxR+1);i++){ const c=j*NX+i; if(solid[c])continue; const b=c*9;
      for(let k=1;k<9;k++){ const ni=i+EX[k],nj=j+EY[k]; if(ni<0||ni>=NX||nj<0||nj>=NY)continue;
        if(solid[nj*NX+ni]){ const mom=f[b+k]+f[b+OPP[k]]; Fx+=EX[k]*mom; Fy+=EY[k]*mom; } } }
  LB.Fx=Fx; LB.Fy=Fy;
  const norm=0.5*LB.uin*LB.uin*LB.Hfront;
  measCd=measCd*0.9 + (Fx/norm)*0.1;
  measCl=measCl*0.9 + (-Fy/norm)*0.1;     // canvas y is down, so lift (up) = -Fy
}
function lbSampleVel(x,y){
  const NX=LB.NX,NY=LB.NY,CS=LB.CS;
  const gx=x/CS-0.5, gy=y/CS-0.5; let i=Math.floor(gx), j=Math.floor(gy);
  if(i<0||i>=NX-1||j<0||j>=NY-1) return {ux:LB.uin,uy:0,sp:LB.uin,solid:false};
  const fx=gx-i, fy=gy-j, s=LB.solid, ux=LB.ux, uy=LB.uy;
  const c00=j*NX+i,c10=c00+1,c01=c00+NX,c11=c01+1;
  const vx=(1-fx)*(1-fy)*ux[c00]+fx*(1-fy)*ux[c10]+(1-fx)*fy*ux[c01]+fx*fy*ux[c11];
  const vy=(1-fx)*(1-fy)*uy[c00]+fx*(1-fy)*uy[c10]+(1-fx)*fy*uy[c01]+fx*fy*uy[c11];
  return {ux:vx,uy:vy,sp:Math.hypot(vx,vy),solid:!!(s[c00]||s[c10]||s[c01]||s[c11])};
}

/* ============================ Colours ============================ */
function jet(t){ t=isFinite(t)?Math.max(0,Math.min(1,t)):0.5;
  return [Math.max(0,Math.min(1,1.5-Math.abs(4*t-3))),
          Math.max(0,Math.min(1,1.5-Math.abs(4*t-2))),
          Math.max(0,Math.min(1,1.5-Math.abs(4*t-1)))]; }
function jetCss(t){ const c=jet(t); return 'rgb('+(c[0]*255|0)+','+(c[1]*255|0)+','+(c[2]*255|0)+')'; }
function ratioColor(r){ return jetCss((r-0.7)/(2.0-0.7)); }      // streamlines by SPEED (blue slow -> red fast)
function cpToColor(cp){ return jetCss((cp+1.5)/2.5); }            // surfaces/pressure (red high -> blue low)
function getCss(v){return getComputedStyle(document.documentElement).getPropertyValue(v).trim();}

/* ============================ Streamlines (integrated through the LBM field) ============================ */
const NSTREAM=90;
function drawStreamlines(){
  if(!LB.inited) return;
  tctx.lineCap='round'; tctx.lineJoin='round';
  const step=4, maxSteps=Math.ceil((TW+80)/step)+60;
  for(let i=0;i<NSTREAM;i++){
    let x=2, y=(i+0.5)/NSTREAM*TH; const pts=[{x,y,r:1}];
    for(let s=0;s<maxSteps;s++){
      const v=lbSampleVel(x,y);
      if(v.solid) break;                                 // stop exactly at the solid surface
      let vx=v.ux+LB.uin*0.09, vy=v.uy;                  // small downstream bias: lines drift through the
      const mag=Math.hypot(vx,vy)||1e-6;                 // wake instead of stalling (no more "giant void")
      x+=vx/mag*step; y+=vy/mag*step;
      pts.push({x,y,r:v.sp/LB.uin});
      if(x>TW+4||y<-30||y>TH+30) break;
    }
    if(pts.length<2) continue;
    tctx.lineWidth=1.15; tctx.globalAlpha=0.85;
    for(let k=1;k<pts.length;k++){ const a=pts[k-1], b=pts[k];
      const col = state.overlays.pressure ? cpToColor(1-b.r*b.r) : ratioColor(b.r);
      tctx.strokeStyle=col; tctx.beginPath(); tctx.moveTo(a.x,a.y); tctx.lineTo(b.x,b.y); tctx.stroke(); }
    tctx.globalAlpha=0.85; tctx.lineWidth=1.4; tctx.strokeStyle='rgba(224,242,255,0.6)';
    tctx.setLineDash([2.5,22]); tctx.lineDashOffset=-((flowPhase+i*5)%24.5);
    tctx.beginPath(); tctx.moveTo(pts[0].x,pts[0].y); for(let k=1;k<pts.length;k++) tctx.lineTo(pts[k].x,pts[k].y);
    tctx.stroke(); tctx.setLineDash([]);
  }
  tctx.globalAlpha=1;
}

/* ============================ Body ============================ */
function drawShape(){
  const pts=getSilhouette();
  tctx.beginPath(); tctx.moveTo(pts[0].x,pts[0].y);
  for(const p of pts) tctx.lineTo(p.x,p.y); tctx.closePath();
  tctx.fillStyle=getCss('--shape-fill');
  tctx.strokeStyle=getCss('--shape-stroke');
  tctx.lineWidth=1.5; tctx.fill(); tctx.stroke();
}
function drawField(){   // continuous flow field so there is never empty "void" space
  if(!LB.inited) return;
  const NX=LB.NX,NY=LB.NY,CS=LB.CS,sld=LB.solid,ux=LB.ux,uy=LB.uy;
  tctx.globalAlpha=0.4;
  for(let j=0;j<NY;j++)for(let i=0;i<NX;i++){ const c=j*NX+i; if(sld[c])continue;
    const sp=Math.hypot(ux[c],uy[c])/LB.uin;
    tctx.fillStyle = state.overlays.pressure ? cpToColor(1-sp*sp) : ratioColor(sp);
    tctx.fillRect(i*CS,j*CS,CS+1,CS+1); }
  tctx.globalAlpha=1;
}
function drawGrid(){ tctx.save(); tctx.strokeStyle='rgba(255,255,255,0.05)'; tctx.lineWidth=1;
  for(let x=0;x<TW;x+=40){tctx.beginPath();tctx.moveTo(x,0);tctx.lineTo(x,TH);tctx.stroke();}
  for(let y=0;y<TH;y+=40){tctx.beginPath();tctx.moveTo(0,y);tctx.lineTo(TW,y);tctx.stroke();} tctx.restore(); }
function drawBoundary(){ const pts=getSilhouette(); tctx.save(); tctx.strokeStyle='rgba(250,204,21,0.5)'; tctx.lineWidth=5; tctx.lineJoin='round';
  tctx.beginPath(); tctx.moveTo(pts[0].x,pts[0].y); for(const p of pts) tctx.lineTo(p.x,p.y); tctx.closePath(); tctx.stroke(); tctx.restore(); }
function drawForces(){
  const dl=Math.min(130,20+Math.log10(1+Math.abs(phys.Fd)*1000)*30);
  arrow(cx,cy,cx-dl,cy,getCss('--drag'),'Fd');
  if(Math.abs(phys.Cl)>1e-4){ const ll=Math.min(120,15+Math.log10(1+Math.abs(phys.Fl)*1000)*30);
    arrow(cx,cy,cx,cy+(phys.Cl>0?-1:1)*ll,getCss('--lift'),'Fl'); }
}
function arrow(x0,y0,x1,y1,col,label){
  tctx.save(); tctx.strokeStyle=col; tctx.fillStyle=col; tctx.lineWidth=3;
  tctx.beginPath(); tctx.moveTo(x0,y0); tctx.lineTo(x1,y1); tctx.stroke();
  const ang=Math.atan2(y1-y0,x1-x0), hs=8;
  tctx.beginPath(); tctx.moveTo(x1,y1);
  tctx.lineTo(x1-hs*Math.cos(ang-0.4),y1-hs*Math.sin(ang-0.4));
  tctx.lineTo(x1-hs*Math.cos(ang+0.4),y1-hs*Math.sin(ang+0.4));
  tctx.closePath(); tctx.fill();
  if(state.overlays.labels){ tctx.font='11px system-ui'; tctx.fillText(label,x1+4,y1-4); }
  tctx.restore();
}
function drawCredit(){ tctx.save(); tctx.font='11px system-ui'; tctx.fillStyle='rgba(148,163,184,0.65)';
  tctx.fillText('Built by Kayan Shah · wind-tunnel.kayanshah.com', 10, TH-8); tctx.restore(); }
function drawLabels(){ tctx.save(); tctx.font='12px system-ui'; tctx.fillStyle='rgba(203,213,225,0.85)';
  tctx.fillText('Freestream',12,18); tctx.fillText('Stagnation',cx-Rpx-78,cy+4); tctx.fillText('Wake',cx+Rpx+10,cy+4); tctx.restore(); }

/* ============================ Graphs ============================ */
function drawGraph(){ gctx.clearRect(0,0,260,140); gctx.fillStyle='#0d1320'; gctx.fillRect(0,0,260,140);
  if(state.gtab==='cp') graphCp(); else if(state.gtab==='forces') graphForces(); else if(state.gtab==='profile') graphProfile(); else graphPolar(); }
function gAxis(){ gctx.strokeStyle='#2a3245'; gctx.lineWidth=1; gctx.strokeRect(28,8,222,118); }
function graphCp(){
  gAxis(); const x0=28,x1=250,y0=8,y1=126,cpTop=1,cpBot=-3;
  const cpY=cp=>y0+(cpTop-cp)/(cpTop-cpBot)*(y1-y0);
  gctx.strokeStyle='#475569'; gctx.setLineDash([3,3]); gctx.beginPath(); gctx.moveTo(x0,cpY(0)); gctx.lineTo(x1,cpY(0)); gctx.stroke(); gctx.setLineDash([]);
  if(LB.inited){
    // measured Cp = 1-(v/U)^2 sampled just outside the surface, around the perimeter
    const pts=getSilhouette(), N=pts.length, gp=[];
    for(let n=0;n<=N;n++){ const p=pts[n%N];
      let nx=p.x-cx, ny=p.y-cy; const nl=Math.hypot(nx,ny)||1; nx/=nl; ny/=nl;
      const v=lbSampleVel(p.x+nx*LB.CS*1.6, p.y+ny*LB.CS*1.6);
      let cp=1-Math.pow(v.sp/LB.uin,2); cp=Math.max(cpBot,Math.min(cpTop,cp));
      gp.push([x0+(n/N)*(x1-x0),cpY(cp)]); }
    gctx.beginPath(); gctx.moveTo(gp[0][0],cpY(0)); for(const p of gp) gctx.lineTo(p[0],p[1]); gctx.lineTo(gp[gp.length-1][0],cpY(0)); gctx.closePath();
    gctx.fillStyle='rgba(96,165,250,0.2)'; gctx.fill();
    gctx.beginPath(); gctx.moveTo(gp[0][0],gp[0][1]); for(const p of gp) gctx.lineTo(p[0],p[1]); gctx.strokeStyle='#60a5fa'; gctx.lineWidth=1.4; gctx.stroke();
  }
  gctx.fillStyle='#9ca3af'; gctx.font='9px system-ui';
  gctx.fillText('+1',8,cpY(1)+3); gctx.fillText('0',14,cpY(0)+3); gctx.fillText('-3',10,cpY(-3)+3);
  gctx.fillText('Cp around surface (measured from flow)',x0,138);
}
function graphForces(){
  gAxis(); const vals=[['Fd',Math.abs(phys.Fd),'#f87171'],['Fl',Math.abs(phys.Fl),'#4ade80'],['q/100',phys.q/100,'#60a5fa']];
  const max=Math.max(vals[0][1],vals[1][1],vals[2][1],1e-6), baseY=118,bw=44,gap=22,x0=48;
  vals.forEach((v,i)=>{ const h=(v[1]/max)*100, x=x0+i*(bw+gap);
    gctx.fillStyle=v[2]; gctx.fillRect(x,baseY-h,bw,h);
    gctx.fillStyle='#cbd5e1'; gctx.font='9px system-ui'; gctx.fillText(v[0],x+6,baseY+11); gctx.fillText(v[1].toFixed(2),x,baseY-h-3); });
  gctx.fillStyle='#9ca3af'; gctx.font='9px system-ui'; gctx.fillText('N / N / Pa',2,12);
}
function graphProfile(){
  const pts=getSilhouette(); let minx=1e9,miny=1e9,maxx=-1e9,maxy=-1e9;
  for(const p of pts){minx=Math.min(minx,p.x);miny=Math.min(miny,p.y);maxx=Math.max(maxx,p.x);maxy=Math.max(maxy,p.y);}
  const bw=maxx-minx||1, bh=maxy-miny||1, scl=Math.min(200/bw,90/bh);
  const ox=130-((minx+maxx)/2)*scl, oy=70-((miny+maxy)/2)*scl;
  gctx.beginPath(); pts.forEach((p,i)=>{const X=ox+p.x*scl,Y=oy+p.y*scl; i?gctx.lineTo(X,Y):gctx.moveTo(X,Y);}); gctx.closePath();
  gctx.fillStyle='#1e3a5f'; gctx.strokeStyle='#63b3ed'; gctx.lineWidth=1.2; gctx.fill(); gctx.stroke();
  gctx.strokeStyle='#9ca3af'; gctx.fillStyle='#9ca3af'; gctx.lineWidth=1.5;
  gctx.beginPath(); gctx.moveTo(8,20); gctx.lineTo(40,20); gctx.stroke();
  gctx.beginPath(); gctx.moveTo(40,20); gctx.lineTo(34,17); gctx.lineTo(34,23); gctx.closePath(); gctx.fill();
  gctx.font='9px system-ui'; gctx.fillText('U',12,16);
}
function graphPolar(){
  if(state.obj!=='stl'){ gctx.fillStyle='#9ca3af'; gctx.font='11px system-ui';
    gctx.fillText('N/A — polar needs a',56,62); gctx.fillText('lifting body (load STL)',54,78); return; }
  gAxis(); const x0=30,x1=248,y0=10,y1=124; const pts=[]; let cdMax=1e-3,clMax=1e-3,clMin=0;
  for(let al=-90;al<=90;al+=2){ const {Cd,Cl}=getCoeffs('stl',al,phys.Re,phys.Mach);
    pts.push({Cd,Cl,al}); cdMax=Math.max(cdMax,Cd); clMax=Math.max(clMax,Cl); clMin=Math.min(clMin,Cl); }
  const X=cd=>x0+(cd/cdMax)*(x1-x0), Y=cl=>y1-((cl-clMin)/(clMax-clMin))*(y1-y0);
  gctx.beginPath(); pts.forEach((p,i)=>{const xx=X(p.Cd),yy=Y(p.Cl); i?gctx.lineTo(xx,yy):gctx.moveTo(xx,yy);});
  gctx.strokeStyle='#60a5fa'; gctx.lineWidth=1.5; gctx.stroke();
  let best=null,bld=-1; for(const p of pts){ if(p.Cd>0){ const ld=p.Cl/p.Cd; if(ld>bld){bld=ld;best=p;} } }
  if(best){ gctx.strokeStyle='#f59e0b'; gctx.setLineDash([4,3]); gctx.beginPath(); gctx.moveTo(X(0),Y(0)); gctx.lineTo(X(best.Cd),Y(best.Cl)); gctx.stroke(); gctx.setLineDash([]);
    gctx.fillStyle='#f59e0b'; gctx.beginPath(); gctx.arc(X(best.Cd),Y(best.Cl),3,0,Math.PI*2); gctx.fill(); }
  gctx.fillStyle='#ef4444'; gctx.beginPath(); gctx.arc(X(phys.Cd),Y(phys.Cl),3.2,0,Math.PI*2); gctx.fill();
  gctx.fillStyle='#9ca3af'; gctx.font='9px system-ui'; gctx.fillText('Cl vs Cd  (red=current)',x0,138);
}

/* ============================ Render loop ============================ */
let rafId=null, tick=0, flowPhase=0;
function frame(){
  bodyParams();
  if(!LB.inited) lbInit();
  const key=state.obj+'|'+state.size+'|'+state.aoa+'|'+state.zoomIdx+'|'+(stlContour?stlContour.length:0);
  if(key!==LB.geomKey){ lbBuildMask(); LB.geomKey=key; }
  lbUpdateTau();
  if(!state.paused){
    tick++; flowPhase += Math.min(Upx,6)*1.6;
    for(let s=0;s<LB_SUBSTEPS;s++) lbStep();
    lbForces();
    // NaN guard: if the solver ever diverges, reset the field
    const midC=(LB.NY>>1)*LB.NX+(LB.NX>>1);
    if(!isFinite(LB.rho[midC])||!isFinite(LB.ux[midC])||!isFinite(LB.ux[LB.NX+1])){ lbInit(); lbBuildMask(); LB.geomKey=key; }
  }
  computePhysics();
  tctx.clearRect(0,0,TW,TH); tctx.fillStyle=getCss('--tunnel'); tctx.fillRect(0,0,TW,TH);
  drawField();
  if(state.overlays.grid) drawGrid();
  if(state.overlays.streamlines) drawStreamlines();
  drawShape();
  if(state.overlays.boundary) drawBoundary();
  if(state.overlays.forces) drawForces();
  if(state.overlays.labels) drawLabels();
  drawCredit();
  drawGraph();
  updateReadouts();
  rafId=requestAnimationFrame(frame);
}

/* ============================ Readouts ============================ */
function set(id,v){const e=document.getElementById(id); if(e) e.textContent=v;}
function fmtRe(re){ if(re>=1e6) return (re/1e6).toFixed(2)+'M'; if(re>=1e3) return (re/1e3).toFixed(1)+'k'; return re.toFixed(0); }
function updateReadouts(){
  const v=uSpeed(phys.V), fd=uForce(phys.Fd), fl=uForce(phys.Fl), q=uPress(phys.q);
  const fdec=state.units==='SI'?3:4, qdec=state.units==='SI'?1:4;
  set('b_v',v.toFixed(1)); set('b_mach',phys.Mach.toFixed(2)); set('b_re',fmtRe(phys.Re));
  set('b_cd',phys.Cd.toFixed(3)); set('b_cl',phys.Cl.toFixed(3));
  set('b_fd',fd.toFixed(fdec)); set('b_fl',fl.toFixed(fdec)); set('b_ld',phys.LD.toFixed(2));
  set('m_v',v.toFixed(1)); set('m_re',fmtRe(phys.Re)); set('m_cd',phys.Cd.toFixed(3)); set('m_cl',phys.Cl.toFixed(3));
  set('m_fd',fd.toFixed(fdec)); set('m_fl',fl.toFixed(fdec)); set('m_q',q.toFixed(qdec)); set('m_ld',phys.LD.toFixed(2));
  document.getElementById('stall').classList.toggle('show', state.stallWarnings && state.obj==='stl' && phys.stall);
  /* STALL SPEED WARNING (restore with calculator panel)
  const belowStall=stallSpeedWarnActive && state.stallWarnings && isFinite(stallVs) && phys.V<stallVs;
  const sw=document.getElementById('stallSpeedWarn');
  sw.classList.toggle('show',belowStall);
  if(belowStall) sw.textContent='BELOW STALL SPEED — Vs = '+stallVs.toFixed(1)+' m/s';
  */
  renderEquations(); renderCoach();
}
function applyUnitLabels(){
  set('bl_v','V ('+lblSpeed()+')'); set('bl_fd','Fd ('+lblForce()+')'); set('bl_fl','Fl ('+lblForce()+')');
  set('mu_v',lblSpeed()); set('mu_fd',lblForce()); set('mu_fl',lblForce()); set('mu_q',lblPress());
}
function renderEquations(){
  if(typeof katex==='undefined') return;
  try{
    kx('eq_re',`Re=\\frac{\\rho V D}{\\mu}=\\frac{1.225\\cdot ${phys.V.toFixed(1)}\\cdot ${phys.D.toFixed(4)}}{1.81\\times10^{-5}}=${phys.Re.toExponential(2)}`);
    kx('eq_q',`q=\\tfrac12\\rho V^2=${phys.q.toFixed(1)}\\,\\text{Pa},\\quad M=\\tfrac{V}{a}=${phys.Mach.toFixed(2)}`);
    kx('eq_fd',`F_d=qAC_d=${phys.q.toFixed(1)}\\cdot ${phys.A.toExponential(2)}\\cdot ${phys.Cd.toFixed(3)}=${phys.Fd.toFixed(3)}\\,\\text{N}`);
    kx('eq_fl',`F_l=qAC_l=${phys.q.toFixed(1)}\\cdot ${phys.A.toExponential(2)}\\cdot ${phys.Cl.toFixed(3)}=${phys.Fl.toFixed(3)}\\,\\text{N}`);
  }catch(e){}
}
function kx(id,tex){ katex.render(tex,document.getElementById(id),{throwOnError:false,displayMode:false}); }
function renderCoach(){
  const tips=[];
  const V2=Math.min(515,state.speed+10), q2=0.5*RHO*V2*V2, fd2=q2*phys.A*phys.Cd;
  tips.push(`Increasing speed by 10 m/s (to ${V2} m/s) raises drag force by ${(fd2-phys.Fd).toFixed(3)} N.`);
  if(phys.Mach>=1) tips.push('Flow is supersonic (M&ge;1) — wave drag dominates.');
  else if(phys.Mach>0.8) tips.push('Transonic regime — compressibility is raising drag sharply.');
  if(state.obj==='stl'){
    if(phys.stall) tips.push(`Stalled — past ${ASTALL}&deg; the flow has separated; lift is dropping and drag is climbing.`);
    else if(Math.abs(state.aoa)>ASTALL-4) tips.push('Approaching stall (~'+ASTALL+'&deg;).');
    else tips.push(`Lift is in the linear (attached) range; L/D = ${phys.LD.toFixed(2)}.`);
  } else tips.push('A sphere is symmetric — angle of attack does not change its coefficients. Load an STL for AoA / stall effects.');
  if(phys.Re>3e5) tips.push('Reynolds number is in the turbulent regime.'); else tips.push('Reynolds number is subcritical / laminar-ish.');
  document.getElementById('coach').innerHTML=tips.map(t=>`<div class="tip">${t}</div>`).join('');
}

/* ============================ STL parsing ============================ */
function parseSTL(buffer){
  const dv=new DataView(buffer), u8=new Uint8Array(buffer); let isAscii=false;
  if(u8.length>5){ const head=String.fromCharCode(u8[0],u8[1],u8[2],u8[3],u8[4]).toLowerCase();
    if(head==='solid'){ if(u8.length>=84){ const tc=dv.getUint32(80,true); if(84+tc*50!==u8.length) isAscii=true; } else isAscii=true; } }
  const tris=[];
  if(isAscii){ const text=new TextDecoder().decode(u8); const re=/vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)/g; const verts=[]; let m;
    while((m=re.exec(text))!==null) verts.push({x:parseFloat(m[1]),y:parseFloat(m[2]),z:parseFloat(m[3])});
    for(let i=0;i+2<verts.length;i+=3) tris.push([verts[i],verts[i+1],verts[i+2]]); }
  else { const tc=dv.getUint32(80,true); let off=84;
    for(let i=0;i<tc;i++){ off+=12; const v=[];
      for(let j=0;j<3;j++){ v.push({x:dv.getFloat32(off,true),y:dv.getFloat32(off+4,true),z:dv.getFloat32(off+8,true)}); off+=12; } off+=2; tris.push(v); } }
  if(tris.length===0) throw new Error('No triangles found');
  return tris;
}
function computeBounds(tris){ const min={x:Infinity,y:Infinity,z:Infinity}, max={x:-Infinity,y:-Infinity,z:-Infinity};
  for(const t of tris) for(const v of t){ min.x=Math.min(min.x,v.x);min.y=Math.min(min.y,v.y);min.z=Math.min(min.z,v.z);
    max.x=Math.max(max.x,v.x);max.y=Math.max(max.y,v.y);max.z=Math.max(max.z,v.z); } return {min,max}; }
function sliceMesh(tris,axis,value){ const segs=[]; const o=axis==='x'?['y','z']:axis==='y'?['x','z']:['x','y'];
  for(const t of tris){ const inter=[];
    for(let e=0;e<3;e++){ const a=t[e], b=t[(e+1)%3], da=a[axis]-value, db=b[axis]-value;
      if((da<=0&&db>0)||(da>0&&db<=0)){ const tt=da/(da-db); inter.push({u:a[o[0]]+(b[o[0]]-a[o[0]])*tt, v:a[o[1]]+(b[o[1]]-a[o[1]])*tt}); } }
    if(inter.length===2) segs.push([inter[0],inter[1]]); } return segs; }
function buildContour(segs){
  if(segs.length<2) return null; const used=new Array(segs.length).fill(false), poly=[];
  used[0]=true; poly.push({x:segs[0][0].u,y:segs[0][0].v}); poly.push({x:segs[0][1].u,y:segs[0][1].v}); let tip=segs[0][1];
  for(let iter=0;iter<segs.length;iter++){ let best=-1,bd=Infinity,bp=null;
    for(let i=0;i<segs.length;i++){ if(used[i])continue; for(const ep of [0,1]){ const p=segs[i][ep], d=(p.u-tip.u)**2+(p.v-tip.v)**2; if(d<bd){bd=d;best=i;bp=segs[i][1-ep];} } }
    if(best<0) break; used[best]=true; poly.push({x:bp.u,y:bp.v}); tip=bp; }
  if(poly.length<4) return null;
  let minx=Infinity,miny=Infinity,maxx=-Infinity,maxy=-Infinity;
  for(const p of poly){minx=Math.min(minx,p.x);miny=Math.min(miny,p.y);maxx=Math.max(maxx,p.x);maxy=Math.max(maxy,p.y);}
  const ccx=(minx+maxx)/2, ccy=(miny+maxy)/2, half=Math.max(maxx-minx,maxy-miny)/2||1;
  return poly.map(p=>({x:(p.x-ccx)/half, y:(p.y-ccy)/half}));
}
function reslice(){
  if(!stlTris) return;
  const planeSel=document.getElementById('slicePlane').value, axis=planeSel==='xz'?'y':planeSel==='xy'?'z':'x';
  const offPct=parseFloat(document.getElementById('sliceOff').value)/100;
  const lo=stlBounds.min[axis], hi=stlBounds.max[axis], mid=(lo+hi)/2, span=hi-lo;
  try{
    const segs=sliceMesh(stlTris,axis,mid+offPct*span), contour=buildContour(segs);
    if(!contour||contour.length<4) throw new Error('Contour too small');
    stlContour=contour; stlHalfW=0; stlHalfH=0;
    for(const p of contour){ stlHalfW=Math.max(stlHalfW,Math.abs(p.x)); stlHalfH=Math.max(stlHalfH,Math.abs(p.y)); }
    stlHalfW=stlHalfW||1; stlHalfH=stlHalfH||1;
    document.getElementById('dropzone').classList.remove('err');
  }catch(e){ stlContour=null; showStlError('Slice produced no valid contour at this offset.'); }
}
function showStlError(msg){ const dz=document.getElementById('dropzone'); dz.classList.add('err');
  dz.innerHTML='STL error: '+msg+' (using sphere)<input type="file" id="stlfile" accept=".stl" style="display:none">'; rebindFileInput(); }
function loadSTL(file){
  const reader=new FileReader();
  reader.onload=function(ev){
    try{ stlTris=parseSTL(ev.target.result); stlBounds=computeBounds(stlTris); reslice();
      if(stlContour){ state.modelName=file.name.replace(/\.stl$/i,'')||'model';
        const dz=document.getElementById('dropzone'); dz.classList.remove('err');
        dz.innerHTML=`Loaded: ${file.name} (${stlTris.length} triangles)<input type="file" id="stlfile" accept=".stl" style="display:none">`;
        rebindFileInput(); document.getElementById('sliceCtl').classList.add('show'); selectObj('stl');
        if(three3d_ren) build3DMesh(); }
    }catch(err){ stlTris=null; stlContour=null; showStlError(err.message||'parse failed'); selectObj('sphere'); }
  };
  reader.onerror=function(){ showStlError('could not read file'); };
  reader.readAsArrayBuffer(file);
}

/* ============================ UI wiring ============================ */
function pushUndo(){ undoStack.push({speed:state.speed,size:state.size,aoa:state.aoa,obj:state.obj}); if(undoStack.length>30) undoStack.shift(); redoStack=[]; }
function applySnapshot(s){ state.speed=s.speed; state.size=s.size; state.aoa=s.aoa; state.obj=s.obj; syncControls(); }
function syncControls(){
  setControlsUI(); applyUnitLabels();
  document.querySelectorAll('.objtab').forEach(b=>b.classList.toggle('on',b.dataset.obj===state.obj));
  document.getElementById('sliceCtl').classList.toggle('show', state.obj==='stl' && !!stlTris);
  if(state.obj!=='stl') state.modelName='sphere';
  document.getElementById('btn3d').disabled=!(state.obj==='stl'&&!!stlTris);
  highlightCmp();
}
function selectObj(obj){ state.obj=obj; syncControls(); }
const spSpeed=document.getElementById('sp_speed'), spSize=document.getElementById('sp_size'), spAoa=document.getElementById('sp_aoa');
const numSpeed=document.getElementById('num_speed'), numSize=document.getElementById('num_size'), numAoa=document.getElementById('num_aoa');
function setControlsUI(){
  spSpeed.value=state.speed; numSpeed.value=state.speed;
  spSize.value=state.size;   numSize.value=state.size;
  spAoa.value=state.aoa;     numAoa.value=state.aoa;
}
function clampVal(v,min,max,def){ v=Math.round(Number(v)); if(!isFinite(v)) return def; return Math.max(min,Math.min(max,v)); }
// sliders
spSpeed.addEventListener('input',()=>{pushUndo();state.speed=+spSpeed.value;numSpeed.value=state.speed;});
spSize.addEventListener('input',()=>{pushUndo();state.size=+spSize.value;numSize.value=state.size;});
spAoa.addEventListener('input',()=>{pushUndo();state.aoa=+spAoa.value;numAoa.value=state.aoa;if(three3d_mesh)color3DMesh();});
// number inputs (live update while typing, validate/clamp on commit)
function wireNum(num,sld,min,max,setter){
  num.addEventListener('input',()=>{ const v=Number(num.value); if(isFinite(v)&&v>=min&&v<=max){ num.classList.remove('bad'); setter(Math.round(v)); sld.value=Math.round(v); } else num.classList.add('bad'); });
  num.addEventListener('change',()=>{ const v=clampVal(num.value,min,max,setter()); pushUndo(); setter(v); sld.value=v; num.value=v; num.classList.remove('bad'); });
}
wireNum(numSpeed,spSpeed,1,515,v=>{ if(v!==undefined)state.speed=v; return state.speed; });
wireNum(numSize,spSize,20,600,v=>{ if(v!==undefined)state.size=v; return state.size; });
wireNum(numAoa,spAoa,-90,90,v=>{ if(v!==undefined)state.aoa=v; return state.aoa; });
numAoa.addEventListener('input',()=>{ if(three3d_mesh)color3DMesh(); });
numAoa.addEventListener('change',()=>{ if(three3d_mesh)color3DMesh(); });
document.querySelectorAll('.objtab').forEach(b=>{ b.addEventListener('click',()=>{ pushUndo();
  if(b.dataset.obj==='stl' && !stlTris) document.getElementById('stlfile').click(); selectObj(b.dataset.obj); }); });
document.querySelectorAll('.ovbtn[data-ov]').forEach(b=>{ b.addEventListener('click',()=>{ const k=b.dataset.ov; state.overlays[k]=!state.overlays[k]; b.classList.toggle('on',state.overlays[k]); }); });
document.querySelectorAll('.gtab').forEach(b=>{ b.addEventListener('click',()=>{ state.gtab=b.dataset.g; document.querySelectorAll('.gtab').forEach(x=>x.classList.toggle('on',x===b)); }); });
document.querySelectorAll('.lpanel .lhead').forEach(h=>{ h.addEventListener('click',()=>h.parentElement.classList.toggle('open')); });

const dropzone=document.getElementById('dropzone');
function rebindFileInput(){ const inp=document.getElementById('stlfile'); inp.addEventListener('change',e=>{ if(e.target.files[0]) loadSTL(e.target.files[0]); }); }
rebindFileInput();
dropzone.addEventListener('click',e=>{ if(e.target.tagName!=='INPUT') document.getElementById('stlfile').click(); });
dropzone.addEventListener('dragover',e=>{e.preventDefault();dropzone.classList.add('drag');});
dropzone.addEventListener('dragleave',()=>dropzone.classList.remove('drag'));
dropzone.addEventListener('drop',e=>{ e.preventDefault(); dropzone.classList.remove('drag'); if(e.dataTransfer.files[0]) loadSTL(e.dataTransfer.files[0]); });
document.getElementById('slicePlane').addEventListener('change',reslice);
document.getElementById('sliceOff').addEventListener('input',e=>{ set('sliceOffVal',e.target.value); reslice(); });

/* ============================ Filenames ============================ */
function modelLabel(){ return state.obj==='stl' ? (state.modelName||'model') : 'sphere'; }
function slug(s){ return String(s).replace(/[^a-z0-9._-]+/gi,'_').replace(/^_+|_+$/g,'') || 'model'; }
function stamp(){ const d=new Date(), p=n=>String(n).padStart(2,'0');
  return d.getFullYear()+p(d.getMonth()+1)+p(d.getDate())+'_'+p(d.getHours())+p(d.getMinutes())+p(d.getSeconds()); }
function fname(ext){ return 'windtunnel_'+slug(modelLabel())+'_'+stamp()+'.'+ext; }

/* ============================ Action buttons ============================ */
const btnPause=document.getElementById('btnPause');
btnPause.addEventListener('click',()=>{ state.paused=!state.paused; btnPause.textContent=state.paused?'Play':'Pause'; btnPause.classList.toggle('act',state.paused); });
const btnZoom=document.getElementById('btnZoom');
btnZoom.addEventListener('click',()=>{ state.zoomIdx=(state.zoomIdx+1)%ZOOMS.length; btnZoom.textContent='Zoom '+ZOOMS[state.zoomIdx]+'x'; });
const btnUnits=document.getElementById('btnUnits');
btnUnits.addEventListener('click',()=>{ state.units=state.units==='SI'?'Imperial':'SI'; btnUnits.textContent='Units: '+(state.units==='SI'?'SI':'Imp'); applyUnitLabels(); });
document.getElementById('btnUndo').addEventListener('click',()=>{ if(!undoStack.length)return; redoStack.push({speed:state.speed,size:state.size,aoa:state.aoa,obj:state.obj}); applySnapshot(undoStack.pop()); });
document.getElementById('btnRedo').addEventListener('click',()=>{ if(!redoStack.length)return; undoStack.push({speed:state.speed,size:state.size,aoa:state.aoa,obj:state.obj}); applySnapshot(redoStack.pop()); });
document.getElementById('btnReset').addEventListener('click',doReset);
function doReset(){ pushUndo(); state.speed=60; state.size=75; state.aoa=0; state.obj='sphere'; state.paused=false; btnPause.textContent='Pause'; btnPause.classList.remove('act'); syncControls(); }
document.getElementById('btnRefresh').addEventListener('click',()=>{ lbInit(); lbBuildMask(); LB.geomKey=''; });
const btnStall=document.getElementById('btnStall');
btnStall.addEventListener('click',()=>{ state.stallWarnings=!state.stallWarnings; btnStall.textContent='Stall warning: '+(state.stallWarnings?'On':'Off'); btnStall.classList.toggle('act',state.stallWarnings); });

document.getElementById('btnCsv').addEventListener('click',exportCSV);
function exportCSV(){
  const us=lblSpeed(), uf=lblForce(), up=lblPress();
  const rows=['# Built by Kayan Shah | hi@kayanshah.com | github.com/KayanShah | wind-tunnel.kayanshah.com',
    `V (${us}),Re,Mach,Cd,Cl,Fd (${uf}),Fl (${uf}),q (${up}),L/D`];
  const D=state.size/1000, A=Math.PI*Math.pow(D/2,2);
  for(let V=1;V<=515;V+=2){ const Re=RHO*V*D/MU, Mach=V/ASOUND; const {Cd,Cl}=getCoeffs(state.obj,state.aoa,Re,Mach);
    const q=0.5*RHO*V*V, Fd=q*A*Cd, Fl=q*A*Cl, LD=Cd?Cl/Cd:0;
    rows.push([uSpeed(V).toFixed(2),Re.toFixed(0),Mach.toFixed(3),Cd.toFixed(4),Cl.toFixed(4),uForce(Fd).toFixed(5),uForce(Fl).toFixed(5),uPress(q).toFixed(5),LD.toFixed(3)].join(',')); }
  download(fname('csv'),'text/csv',rows.join('\n'));
}
document.getElementById('btnPng').addEventListener('click',exportPNG);
function exportPNG(){ const a=document.createElement('a'); a.href=tunnel.toDataURL('image/png'); a.download=fname('png'); a.click(); }
document.getElementById('btnPdf').addEventListener('click',generatePDF);
function captureGraph(tab){ const prev=state.gtab; state.gtab=tab; drawGraph(); const d=graph.toDataURL('image/png'); state.gtab=prev; return d; }
function generatePDF(){
  if(!(window.jspdf && window.jspdf.jsPDF)){ alert('PDF library not loaded (offline?). Try the PNG / CSV exports.'); return; }
  const { jsPDF }=window.jspdf;
  const doc=new jsPDF({orientation:'portrait',unit:'pt',format:'a4'});
  const W=595.28, M=40, us=lblSpeed(), uf=lblForce(), up=lblPress(), ul=lblLen();
  doc.setFont('helvetica','bold'); doc.setFontSize(18); doc.setTextColor(20);
  doc.text('Wind Tunnel Report', M, 48);
  doc.setFont('helvetica','normal'); doc.setFontSize(10); doc.setTextColor(110);
  doc.text('Model: '+modelLabel()+'      Generated: '+new Date().toLocaleString()+'      Units: '+state.units, M, 66);
  // stats
  const rows=[
    ['Air speed',uSpeed(phys.V).toFixed(1)+' '+us], ['Mach',phys.Mach.toFixed(2)],
    ['Reynolds number',phys.Re.toFixed(0)], ['Object size',uLen(state.size).toFixed(2)+' '+ul],
    ['Angle of attack',state.aoa+' deg'], ['Stall',phys.stall?'YES (separated)':'no'],
    ['Drag coefficient Cd',phys.Cd.toFixed(3)], ['Lift coefficient Cl',phys.Cl.toFixed(3)],
    ['Drag force Fd',uForce(phys.Fd).toFixed(4)+' '+uf], ['Lift force Fl',uForce(phys.Fl).toFixed(4)+' '+uf],
    ['Dynamic pressure q',uPress(phys.q).toFixed(4)+' '+up], ['L/D ratio',phys.LD.toFixed(2)]
  ];
  doc.setFontSize(10); doc.setTextColor(30); let y=92;
  for(let i=0;i<rows.length;i+=2){
    const left=rows[i], right=rows[i+1];
    doc.setTextColor(110); doc.text(left[0], M, y); if(right) doc.text(right[0], M+280, y);
    doc.setTextColor(20); doc.setFont('helvetica','bold');
    doc.text(String(left[1]), M+150, y); if(right) doc.text(String(right[1]), M+430, y);
    doc.setFont('helvetica','normal'); y+=16;
  }
  // tunnel snapshot
  const timg=tunnel.toDataURL('image/png'); const iw=W-2*M, ih=iw*(tunnel.clientHeight/tunnel.clientWidth);
  let iy=y+8; doc.setTextColor(110); doc.setFontSize(9); doc.text('Tunnel view', M, iy); iy+=6;
  doc.addImage(timg,'PNG',M,iy,iw,ih);
  // four graphs in a 2x2 grid
  const tabs=[['cp','Cp distribution'],['forces','Forces'],['profile','Profile'],['polar','Polar']];
  const gw=(W-2*M-20)/2, gh=gw*(140/260); let gy=iy+ih+18;
  tabs.forEach((t,k)=>{ const gx=M+(k%2)*(gw+20), yy=gy+Math.floor(k/2)*(gh+22);
    doc.setTextColor(110); doc.setFontSize(9); doc.text(t[1], gx, yy);
    doc.addImage(captureGraph(t[0]),'PNG',gx,yy+4,gw,gh); });
  // credit footer
  doc.setFontSize(8); doc.setTextColor(120);
  doc.text('Built by Kayan Shah', M, 822);
  doc.textWithLink('hi@kayanshah.com', M+92, 822, {url:'mailto:hi@kayanshah.com'});
  doc.textWithLink('github.com/KayanShah', M+190, 822, {url:'https://github.com/KayanShah'});
  doc.textWithLink('wind-tunnel.kayanshah.com', M+308, 822, {url:'https://wind-tunnel.kayanshah.com'});
  // Page 2 — 3D surface pressure map (only when the 3D view has been initialised)
  if(three3d_ren&&three3d_scene&&three3d_cam&&stlTris){
    three3d_ren.render(three3d_scene,three3d_cam);
    const img3d=document.getElementById('canvas3d').toDataURL('image/png');
    doc.addPage();
    doc.setFont('helvetica','bold'); doc.setFontSize(16); doc.setTextColor(20);
    doc.text('3D Surface Pressure Distribution (Cp)',M,48);
    doc.setFont('helvetica','normal'); doc.setFontSize(9); doc.setTextColor(110);
    const dir3={'1,0,0':'+X','-1,0,0':'−X','0,1,0':'+Y','0,-1,0':'−Y','0,0,1':'+Z','0,0,-1':'−Z'}[freestreamBase3d.join(',')]||'?';
    doc.text('Model: '+modelLabel()+'  ·  Wind from: '+dir3+'  ·  AoA: '+state.aoa+'°  ·  Scale: '+cpScaleFactor.toFixed(2)+'×',M,63);
    const cv3d=document.getElementById('canvas3d');
    const iw3d=W-2*M;
    const ih3d=Math.min(iw3d*(cv3d.clientHeight||window.innerHeight-74)/(cv3d.clientWidth||window.innerWidth),660);
    doc.addImage(img3d,'PNG',M,72,iw3d,ih3d);
    // Cp gradient legend
    const ly=72+ih3d+10;
    doc.setFontSize(8); doc.setTextColor(110);
    doc.text('Suction (−)',M,ly+8);
    doc.text('Stagnation (+)',M+iw3d-56,ly+8);
    const nS=40,lgX=M+60,lgW=iw3d-116;
    for(let k=0;k<nS;k++){
      const [cr,cg,cb]=jet(k/(nS-1));
      doc.setFillColor(Math.round(cr*255),Math.round(cg*255),Math.round(cb*255));
      doc.rect(lgX+k*lgW/nS,ly,lgW/nS+0.5,6,'F');
    }
    doc.setFontSize(8); doc.setTextColor(120);
    doc.text('Built by Kayan Shah',M,822);
    doc.textWithLink('wind-tunnel.kayanshah.com',M+308,822,{url:'https://wind-tunnel.kayanshah.com'});
  }
  doc.save(fname('pdf'));
}
function download(name,type,data){ const blob=new Blob([data],{type}), url=URL.createObjectURL(blob);
  const a=document.createElement('a'); a.href=url; a.download=name; a.click(); setTimeout(()=>URL.revokeObjectURL(url),1000); }

/* ============================ Context menu + keyboard ============================ */
const ctxmenu=document.getElementById('ctxmenu');
function showCtx(x,y){ ctxmenu.style.left=x+'px'; ctxmenu.style.top=y+'px'; ctxmenu.style.display='block'; }
function hideCtx(){ ctxmenu.style.display='none'; }
[tunnel,graph].forEach(cv=>cv.addEventListener('contextmenu',e=>{ e.preventDefault(); showCtx(e.clientX,e.clientY); }));
ctxmenu.addEventListener('click',e=>{
  const act=e.target.dataset.act;
  if(act==='copy'){ const txt=`V=${uSpeed(phys.V).toFixed(1)}${lblSpeed()} M=${phys.Mach.toFixed(2)} Re=${phys.Re.toFixed(0)} Cd=${phys.Cd.toFixed(3)} Cl=${phys.Cl.toFixed(3)} L/D=${phys.LD.toFixed(2)}`;
    if(navigator.clipboard) navigator.clipboard.writeText(txt).catch(()=>{}); }
  else if(act==='png') exportPNG(); else if(act==='csv') exportCSV();
  else if(act==='grid'){ state.overlays.grid=!state.overlays.grid; document.querySelector('.ovbtn[data-ov="grid"]').classList.toggle('on',state.overlays.grid); }
  else if(act==='reset') doReset(); hideCtx();
});
document.addEventListener('click',e=>{ if(!ctxmenu.contains(e.target)) hideCtx(); });
document.addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT'||e.target.tagName==='SELECT') return;
  if(e.code==='Space'){ e.preventDefault(); btnPause.click(); }
  else if((e.key==='r'||e.key==='R')&&!e.ctrlKey&&!e.metaKey) doReset();
  else if(e.key>='1'&&e.key<='4'){ const g=['cp','forces','profile','polar'][+e.key-1]; state.gtab=g; document.querySelectorAll('.gtab').forEach(x=>x.classList.toggle('on',x.dataset.g===g)); }
  else if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'&&!e.shiftKey){ e.preventDefault(); document.getElementById('btnUndo').click(); }
  else if((e.ctrlKey||e.metaKey)&&((e.key.toLowerCase()==='z'&&e.shiftKey)||e.key.toLowerCase()==='y')){ e.preventDefault(); document.getElementById('btnRedo').click(); }
  else if(e.key==='Escape') hideCtx();
});

/* ============================ Comparison table ============================ */
const CMP=[
  ['sphere','Sphere','0.47 / 0.20','0','1e4 – 1e6','Bluff body; drag crisis above Re 3e5'],
  ['stl','STL (your model)','from cross-section','from AoA','depends on size','Full-range stall model; Cd/Cl scale with thickness & angle']
];
function buildCmp(){ const tb=document.querySelector('#cmpTable tbody');
  tb.innerHTML=CMP.map(r=>`<tr data-obj="${r[0]}"><td>${r[1]}</td><td>${r[2]}</td><td>${r[3]}</td><td>${r[4]}</td><td>${r[5]}</td></tr>`).join(''); highlightCmp(); }
function highlightCmp(){ document.querySelectorAll('#cmpTable tbody tr').forEach(tr=>tr.classList.toggle('cur',tr.dataset.obj===state.obj)); }

/* ============================ Stall speed calculator (commented out, restore with HTML panel above) ============================
const STALL_PRESETS={
  concorde:{W:1841337, rho:1.225, S:358.25, CLmax:1.35},
  light:   {W:10898,   rho:1.225, S:16.16,  CLmax:1.4},
  sphere:  {W:null,    rho:1.225, S:null,    CLmax:null},
  custom:  {W:5000,    rho:1.225, S:20,      CLmax:1.2}
};
let stallVs=NaN, stallSpeedWarnActive=true;

function computeVs(W,rho,S,CLmax){
  if(!W||!S||!CLmax||rho<=0||S<=0||CLmax<=0) return NaN;
  return Math.sqrt(2*W/(rho*S*CLmax));
}
function syncStallPreset(preset){
  const p=STALL_PRESETS[preset], isSphere=preset==='sphere';
  document.getElementById('stall_W').value=p.W||''; document.getElementById('stall_W').disabled=isSphere;
  document.getElementById('stall_rho').value=p.rho;
  document.getElementById('stall_S').value=p.S||''; document.getElementById('stall_S').disabled=isSphere;
  document.getElementById('stall_CLmax').value=p.CLmax||''; document.getElementById('stall_CLmax').disabled=isSphere;
  refreshVs();
}
function refreshVs(){
  const W=parseFloat(document.getElementById('stall_W').value),
        rho=parseFloat(document.getElementById('stall_rho').value),
        S=parseFloat(document.getElementById('stall_S').value),
        CLmax=parseFloat(document.getElementById('stall_CLmax').value);
  const vs=computeVs(W,rho,S,CLmax);
  stallVs=vs;
  document.getElementById('vsDisplay').textContent=isFinite(vs)?vs.toFixed(1):'N/A';
  if(typeof katex!=='undefined'&&document.getElementById('eq_vs')){
    const Wv=isFinite(W)?W.toExponential(3):'W',rv=isFinite(rho)?rho.toFixed(3):'\\rho',
          Sv=isFinite(S)?S.toFixed(2):'S',cv=isFinite(CLmax)?CLmax.toFixed(2):'C_{L,max}',
          vsv=isFinite(vs)?vs.toFixed(1)+'\\,\\text{m/s}':'\\text{N/A}';
    try{ katex.render(`V_s=\\sqrt{\\dfrac{2W}{\\rho S C_{L,max}}}=\\sqrt{\\dfrac{2\\times${Wv}}{${rv}\\times${Sv}\\times${cv}}}=${vsv}`,
      document.getElementById('eq_vs'),{throwOnError:false,displayMode:false}); }catch(e){}
  }
}
document.getElementById('stallPreset').addEventListener('change',e=>syncStallPreset(e.target.value));
['stall_W','stall_rho','stall_S','stall_CLmax'].forEach(id=>{
  document.getElementById(id).addEventListener('input',()=>{ document.getElementById('stallPreset').value='custom'; refreshVs(); });
});
const btnStallSpeedWarn=document.getElementById('btnStallSpeedWarn');
btnStallSpeedWarn.addEventListener('click',()=>{
  stallSpeedWarnActive=!stallSpeedWarnActive;
  btnStallSpeedWarn.textContent='Speed warning: '+(stallSpeedWarnActive?'On':'Off');
  btnStallSpeedWarn.classList.toggle('act',stallSpeedWarnActive);
});
syncStallPreset('concorde');
*/

/* ============================ 3D Pressure Map ============================ */
function build3DMesh(){
  if(!stlTris||typeof THREE==='undefined') return;
  if(!three3d_scene){
    three3d_scene=new THREE.Scene();
    three3d_scene.background=new THREE.Color(0x0d1320);
    three3d_cam=new THREE.PerspectiveCamera(45,2,0.001,100);
  }
  if(three3d_mesh){ three3d_scene.remove(three3d_mesh); three3d_mesh.geometry.dispose(); three3d_mesh=null; }
  const n=stlTris.length;
  const pos=new Float32Array(n*9), col=new Float32Array(n*9);
  const b=stlBounds;
  const cx_=(b.min.x+b.max.x)/2, cy_=(b.min.y+b.max.y)/2, cz_=(b.min.z+b.max.z)/2;
  const ext=Math.max(b.max.x-b.min.x,b.max.y-b.min.y,b.max.z-b.min.z)||1;
  const sc=2/ext;
  for(let i=0;i<n;i++){
    const t=stlTris[i];
    for(let v=0;v<3;v++){
      pos[i*9+v*3  ]=(t[v].x-cx_)*sc;
      pos[i*9+v*3+1]=(t[v].y-cy_)*sc;
      pos[i*9+v*3+2]=(t[v].z-cz_)*sc;
    }
  }
  const geom=new THREE.BufferGeometry();
  geom.setAttribute('position',new THREE.BufferAttribute(pos,3));
  geom.setAttribute('color',new THREE.BufferAttribute(col,3));
  const mat=new THREE.MeshBasicMaterial({vertexColors:true,side:THREE.DoubleSide});
  three3d_mesh=new THREE.Mesh(geom,mat);
  three3d_scene.add(three3d_mesh);
  autoDetectFreestream();
  color3DMesh();
}
function autoDetectFreestream(){
  // Try each of 6 axis directions. The nose/leading edge is a pointed region
  // where few faces are highly aligned with any one direction. The direction
  // where the FEWEST faces score cosT > 0.85 = pointed end = correct freestream.
  if(!three3d_mesh||!stlTris) return;
  const pos=three3d_mesh.geometry.attributes.position.array;
  const n=stlTris.length;
  const step=Math.max(1,Math.floor(n/1000));
  const dirs=[[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]];
  const nx_s=[],ny_s=[],nz_s=[];
  for(let i=0;i<n;i+=step){
    const p0x=pos[i*9],p0y=pos[i*9+1],p0z=pos[i*9+2];
    const p1x=pos[i*9+3],p1y=pos[i*9+4],p1z=pos[i*9+5];
    const p2x=pos[i*9+6],p2y=pos[i*9+7],p2z=pos[i*9+8];
    const e0x=p1x-p0x,e0y=p1y-p0y,e0z=p1z-p0z;
    const e1x=p2x-p0x,e1y=p2y-p0y,e1z=p2z-p0z;
    let nx=e0y*e1z-e0z*e1y,ny=e0z*e1x-e0x*e1z,nz=e0x*e1y-e0y*e1x;
    const nl=Math.hypot(nx,ny,nz)||1;nx/=nl;ny/=nl;nz/=nl;
    const gcx=(p0x+p1x+p2x)/3,gcy=(p0y+p1y+p2y)/3,gcz=(p0z+p1z+p2z)/3;
    if(nx*gcx+ny*gcy+nz*gcz<0){nx=-nx;ny=-ny;nz=-nz;}
    nx_s.push(nx);ny_s.push(ny);nz_s.push(nz);
  }
  let bestDir=[1,0,0],bestCount=Infinity;
  for(const [dx,dy,dz] of dirs){
    let count=0;
    for(let i=0;i<nx_s.length;i++) if(nx_s[i]*dx+ny_s[i]*dy+nz_s[i]*dz>0.85) count++;
    if(count<bestCount){bestCount=count;bestDir=[dx,dy,dz];}
  }
  freestreamBase3d=bestDir;
  const sel=document.getElementById('flow3d-dir');
  if(sel) sel.value=bestDir.join(',');
  updateWindIndicator();
  updateArrow3d();
}
const WIND_ARROWS={'1,0,0':'→','-1,0,0':'←','0,1,0':'↑','0,-1,0':'↓','0,0,1':'⊙','0,0,-1':'⊗'};
function updateWindIndicator(){
  const el=document.getElementById('wind3d-arrow');
  if(el) el.textContent=WIND_ARROWS[freestreamBase3d.join(',')] || '→';
}
function updateArrow3d(){
  if(!three3d_scene||typeof THREE==='undefined') return;
  if(arrow3d){ three3d_scene.remove(arrow3d); arrow3d=null; }
  const [bx,by,bz]=freestreamBase3d;
  // Arrow shows the flow direction (wind travels from source toward the model)
  const dir=new THREE.Vector3(-bx,-by,-bz);
  const origin=new THREE.Vector3(bx*1.85,by*1.85-0.3,bz*1.85);
  arrow3d=new THREE.ArrowHelper(dir,origin,0.75,0x00d4ff,0.22,0.13);
  three3d_scene.add(arrow3d);
}
function color3DMesh(){
  if(!three3d_mesh||!stlTris) return;
  const pos=three3d_mesh.geometry.attributes.position.array;
  const col=three3d_mesh.geometry.attributes.color.array;
  const n=stlTris.length;
  const aoaRad=state.aoa*DEG;
  // Rotate freestream base around Y axis by AoA
  const [bx,by,bz]=freestreamBase3d;
  const fx=bx*Math.cos(aoaRad)-bz*Math.sin(aoaRad);
  const fy=by;
  const fz=bx*Math.sin(aoaRad)+bz*Math.cos(aoaRad);
  // Pass 1: compute Cp per face, track range
  const cpArr=new Float32Array(n);
  let cpMin=Infinity,cpMax=-Infinity;
  for(let i=0;i<n;i++){
    const p0x=pos[i*9],p0y=pos[i*9+1],p0z=pos[i*9+2];
    const p1x=pos[i*9+3],p1y=pos[i*9+4],p1z=pos[i*9+5];
    const p2x=pos[i*9+6],p2y=pos[i*9+7],p2z=pos[i*9+8];
    const e0x=p1x-p0x,e0y=p1y-p0y,e0z=p1z-p0z;
    const e1x=p2x-p0x,e1y=p2y-p0y,e1z=p2z-p0z;
    let nx=e0y*e1z-e0z*e1y,ny=e0z*e1x-e0x*e1z,nz=e0x*e1y-e0y*e1x;
    const nl=Math.hypot(nx,ny,nz)||1;nx/=nl;ny/=nl;nz/=nl;
    const gcx=(p0x+p1x+p2x)/3,gcy=(p0y+p1y+p2y)/3,gcz=(p0z+p1z+p2z)/3;
    if(nx*gcx+ny*gcy+nz*gcz<0){nx=-nx;ny=-ny;nz=-nz;}
    // Full 3D dot product — includes all three normal components
    const cosT=nx*fx+ny*fy+nz*fz;
    const Cp=cosT>0?cosT*cosT:-0.4*Math.abs(cosT);
    cpArr[i]=Cp;
    if(Cp<cpMin)cpMin=Cp;
    if(Cp>cpMax)cpMax=Cp;
  }
  // Pass 2: normalize, then apply cpScaleFactor (<1 = higher contrast, >1 = lower)
  const autoRange=Math.max(cpMax-cpMin,0.01);
  const midCp=(cpMin+cpMax)/2, halfR=(autoRange/2)*cpScaleFactor;
  const effMin=midCp-halfR, effRange=halfR*2;
  for(let i=0;i<n;i++){
    const [r2,g2,b2]=jet(Math.max(0,Math.min(1,(cpArr[i]-effMin)/effRange)));
    for(let v=0;v<3;v++){col[i*9+v*3]=r2;col[i*9+v*3+1]=g2;col[i*9+v*3+2]=b2;}
  }
  three3d_mesh.geometry.attributes.color.needsUpdate=true;
}
function updateCam3(){
  if(!three3d_cam) return;
  const {theta,phi,r}=orb;
  three3d_cam.position.set(r*Math.sin(phi)*Math.sin(theta),r*Math.cos(phi),r*Math.sin(phi)*Math.cos(theta));
  three3d_cam.lookAt(0,0,0);
}
function render3d(){
  const ov=document.getElementById('view3d');
  if(!ov||ov.style.display==='none'){ raf3d=0; return; }
  raf3d=requestAnimationFrame(render3d);
  if(three3d_ren&&three3d_scene&&three3d_cam) three3d_ren.render(three3d_scene,three3d_cam);
}
function open3DView(){
  if(!stlTris||typeof THREE==='undefined') return;
  const ov=document.getElementById('view3d'); ov.style.display='flex';
  const W=window.innerWidth, H=window.innerHeight-74;
  if(!three3d_ren){
    const cv=document.getElementById('canvas3d');
    three3d_ren=new THREE.WebGLRenderer({canvas:cv,antialias:true,preserveDrawingBuffer:true});
    three3d_ren.setPixelRatio(window.devicePixelRatio||1);
    if(!three3d_scene){ build3DMesh(); }
  }
  three3d_ren.setSize(W,H);
  three3d_cam.aspect=W/H; three3d_cam.updateProjectionMatrix();
  updateCam3();
  if(!raf3d) render3d();
}
function close3DView(){
  const ov=document.getElementById('view3d'); if(ov) ov.style.display='none';
  cancelAnimationFrame(raf3d); raf3d=0;
}
document.getElementById('btn3d').addEventListener('click',open3DView);
document.getElementById('close3d').addEventListener('click',close3DView);
document.getElementById('flow3d-dir').addEventListener('change',e=>{
  freestreamBase3d=e.target.value.split(',').map(Number);
  updateWindIndicator(); updateArrow3d();
  if(three3d_mesh) color3DMesh();
});
document.getElementById('cp3d-scale').addEventListener('input',e=>{
  cpScaleFactor=parseFloat(e.target.value);
  document.getElementById('cp3d-scv').textContent=cpScaleFactor.toFixed(2)+'×';
  if(three3d_mesh) color3DMesh();
});
document.addEventListener('keydown',e=>{ if(e.key==='Escape') close3DView(); });
(function(){
  const cv3=document.getElementById('canvas3d');
  cv3.addEventListener('mousedown',e=>{ orb.down=true; orb.ox=e.clientX; orb.oy=e.clientY; orb.ot=orb.theta; orb.op=orb.phi; });
  window.addEventListener('mousemove',e=>{ if(!orb.down) return;
    orb.theta=orb.ot-(e.clientX-orb.ox)*0.008;
    orb.phi=Math.max(0.08,Math.min(Math.PI-0.08,orb.op+(e.clientY-orb.oy)*0.008));
    updateCam3(); });
  window.addEventListener('mouseup',()=>{ orb.down=false; });
  cv3.addEventListener('wheel',e=>{ e.preventDefault();
    orb.r=Math.max(1,Math.min(10,orb.r+e.deltaY*0.005)); updateCam3(); },{passive:false});
  window.addEventListener('resize',()=>{
    if(!three3d_ren||document.getElementById('view3d').style.display==='none') return;
    const W=window.innerWidth,H=window.innerHeight-74;
    three3d_ren.setSize(W,H); three3d_cam.aspect=W/H; three3d_cam.updateProjectionMatrix();
  });
})();

/* ============================ Init ============================ */
buildCmp(); resize(); syncControls(); frame();
})();
