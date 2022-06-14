DisplayNo=1
--Display config
GridN1=4
GridN2=4
FrameC={0,31,0,255}
GridC={0,15,0,255}
TextC={31,31,31,255}
TgtC={0,255,0,255}
--Process config
MargeDist=10
MaxTGT=32
HoldTick=180

iB=input.getBool
iN=input.getNumber
m=math
pi=m.pi
pi2=2*pi
s=m.sin
c=m.cos
t=m.tan
as=m.asin
ac=m.acos
at=m.atan
rads=pi/180
sc=screen
sC=sc.setColor
dT=sc.drawText
dL=sc.drawLine
dC=sc.drawCircle
dR=sc.drawRect
fo=string.format
tb=table
tI=tb.insert
tR=tb.remove
pN=property.getNumber
pB=property.getBool
Scale=pB("Show Map Scale")
ScH=pN("Map Scale Horizontal Position")
ScV=pN("Map Scale Vertical Position")
Marge=pB("Marge close targets")
MapStyle=pN("Target Map Style")
RUnit=pN("Display Range Unit")
Rng={pN("Display Range 1"),pN("Display Range 2"),pN("Display Range 3"),pN("Display Range 4")}
DAMin=pN("Min display azim. angle (deg)")*rads
DAMax=pN("Max display azim. angle (deg)")*rads
DEMin=pN("Min display elev. angle (deg)")*rads
DEMax=pN("Max display elev. angle (deg)")*rads
function distV(V1,V2)
	_=0
	for i=1,3 do
		_=_+(V1[i]-V2[i])^2	
	end
	return _^0.5
end
function outPro(u,v)
	w={}
	w[1]=u[2]*v[3]-u[3]*v[2]
	w[2]=u[3]*v[1]-u[1]*v[3]
	w[3]=u[1]*v[2]-u[2]*v[1]
	return w
end
function AngleToBasis(Q)
	ex={c(Q[1])*s(-Q[2]),c(Q[1])*c(-Q[2]),s(Q[1])}
	ey={c(Q[3])*s(-Q[4]),c(Q[3])*c(-Q[4]),s(Q[3])}
	ez=outPro(ex,ey)
	return {ex,ey,ez}
end
function Mv(M,v)
	V={}
	for i=1,3 do
		_=0
		for j=1,3 do
			_=_+M[j][i]*v[j]
		end
		V[i]=_
	end
	return V
end
function inv(E)
	e1,e2,e3=E[1],E[2],E[3]
	a,b,C,d,e,f,g,h,i=e1[1],e2[1],e3[1],e1[2],e2[2],e3[2],e1[3],e2[3],e3[3]
	det=a*e*i+d*h*C+g*b*f-g*e*C-a*h*f-d*b*i
	E1={(e*i-f*h)/det,(f*g-d*i)/det,(d*h-e*g)/det}
	E2={(C*h-b*i)/det,(a*i-C*g)/det,(b*g-a*h)/det}
	E3={(b*f-C*e)/det,(C*d-a*f)/det,(a*e-b*d)/det}
	return {E1,E2,E3},det
end
function RegisterTGT(tgtV)
	tgtV[4]=0
	tI(TGTD,tgtV)
	if #TGTD>MaxTGT then
		tR(TGTD,1)
	end
end
function OrthToPol(VO)
	VP={}
	VP[1]=distV(VO,{0,0,0})
	VP[2]=at(VO[1],VO[2])
	VP[3]=as(VO[3]/_)
	return VP
end
function dC2(x,y,r,dn)
	for i=1,dn do
		qs,qe=(i-1)*pi2/dn,i*pi2/dn
		dL(x+r*c(qs),y-r*s(qs),x+r*c(qe),y-r*s(qe))
	end
end
function sC2(T)
	sC(T[1],T[2],T[3],T[4])
end
TGTS={}
TGTD={}
w,h=8
RngN=1
isPrO=false

function onTick()
	if MapStyle~=0 then
		PO={iN(4),iN(8),iN(12)}
		QO={iN(16)*pi2,iN(20)*pi2,iN(24)*pi2,iN(28)*pi2}
		es=AngleToBasis(QO)
		Es,_=inv(es)
		cb=iN(32)*pi2
		isPr=iB(32)
		--Input
		for i=1,8 do
			det=iB(i)
			if det then
				for j=1,3 do
					TGTS[i][j]=iN(i*4-(4-j))
				end
				if #TGTD>0 then
					dist,km=distV(TGTD[1],TGTS[i]),1
					for k=1,#TGTD do
						distk=distV(TGTD[k],TGTS[i])
						if distk<dist then
							dist=distk
							km=k
						end
					end
					if dist<MargeDist then
						for j=1,3 do
							TGTD[km][j]=TGTS[i][j]
						end
						TGTD[km][4]=0
					else
						RegisterTGT(TGTS[i])
					end
				else
					RegisterTGT(TGTS[i])
				end
			else
				TGTS[i]={0,0,0}
			end
		end
		--Range setting
		if isPr and not isPrO then
			for i=1,#Rng do
				RngN=(RngN)%#Rng+1
				if Rng[RngN]>0 then break end
			end
		end
		isPrO=isPr

		--Target data tick update
		if #TGTD>0 then
			for k=#TGTD,1,-1 do
				TGTD[k][4]=TGTD[k][4]+1
				if TGTD[k][4]>HoldTick then
					tR(TGTD,k)
				end
			end
		end
	end
end

function onDraw()
	if MapStyle~=0 then
		w=sc.getWidth()
		h=sc.getHeight()-1
		--Generate map plot data
		if MapStyle==1 or MapStyle==2 then
			ScaleH=h/(2*Rng[RngN]*RUnit)
			ScaleV=-ScaleH
		elseif MapStyle==3 then
			ScaleH=h/(2*Rng[RngN]*RUnit)
			ScaleV=-ScaleH
		elseif MapStyle==4 then
			ScaleH=w/(2*m.max(m.abs(DAMin),m.abs(DAMax)))
			if DisplayNo==1 then
				ScaleV=-h/(Rng[RngN]*RUnit)
			else
				ScaleV=-h/(2*m.max(m.abs(DEMin),m.abs(DEMax)))
			end
		else		
		end
		if #TGTD>0 then
			for k=1,#TGTD do
				PTR={}
				for j=1,3 do
					PTR[j]=TGTD[k][j]-PO[j]
				end
				if MapStyle==1 or MapStyle==2 then
					TGTD[k][5]=PTR[1]*ScaleH
					TGTD[k][6]=PTR[2]*ScaleV
				else
					PTr=Mv(Es,PTR)
					if MapStyle==3 then
						TGTD[k][5]=PTr[1]*ScaleH
						if DisplayNo==1 then
							TGTD[k][6]=PTr[2]*ScaleV
						else
							TGTD[k][6]=PTr[3]*ScaleV
						end
					elseif MapStyle==4 then
						PTp=OrthToPol(PTr)
						TGTD[k][5]=PTp[2]*ScaleH
						if DisplayNo==1 then
							TGTD[k][6]=PTp[1]*ScaleV
						else
							TGTD[k][6]=PTp[3]*ScaleV
						end
					end
				end
				TGTD[k][7]=distV(PTR,{0,0,0})
			end
		end
		--Map frame and grid
		sC2(GridC)
		if MapStyle==1 or MapStyle==2 or MapStyle==3 then
			HMin,HMax,VMin,VMax=0,w,h,0
			if MapStyle==2 then screen.drawMap(PO[1],PO[2],2*Rng[RngN]*RUnit/10^3) end
			if GridN1>0 then
				for i=1,GridN1 do
					qc=pi/2
					qi=pi2/GridN1*i
					dL(w/2,h/2,w/2+(h/2)*c(qc+qi),h/2-(h/2)*s(qc+qi))
				end
			end
			for i=1,GridN2 do
				ri=h/2/(GridN2+1)*i
				dC2(w/2,h/2,ri,36)
			end
			sC2(FrameC)
			dC2(w/2,h/2,h/2,36)
		elseif MapStyle==4 then
			HMin,HMax=w/2+DAMin*ScaleH,w/2+DAMax*ScaleH
			if DisplayNo==1 then
				VMin,VMax=h,0
			else
				VMin,VMax=h/2+DEMin*ScaleV,h/2+DEMax*ScaleV
			end
			if GridN1>0 then
				for i=1,GridN1 do
					for j=-1,1,2 do
						if DisplayNo==1 then
							hi=h/(GridN1+1)*i
						else
							hi=h/2+j*h/2/(GridN1+1)*i
						end
						if hi<VMin and hi>VMax then
							dL(HMin,hi,HMax,hi)
						end
					end
				end
			end
			if GridN2>0 then
				for i=1,GridN2 do
					for j=-1,1,2 do
						wi=w/2+j*w/2/(GridN2+1)*i
						if wi>HMin and wi<HMax then
							dL(wi,VMin,wi,VMax)
						end
					end
				end
			end
			sC2(FrameC)
			dL(w/2,VMin,w/2,VMax)
			dR(HMin,VMin,HMax-HMin-1,VMax-VMin)
			if DisplayNo==2 then dL(HMin,h/2,HMax,h/2) end
		end
		--Range
		if Scale then
			sC2(TextC)
			tw=string.len(Rng[RngN])-2
			dT((w-5*tw)*ScH,(h-5)*ScV,fo("%d",Rng[RngN]))
		end
		--Targets
		if #TGTD>0 then
			for k=1,#TGTD do
				if TGTD[k][7]<Rng[RngN]*RUnit then
					TgtCk={TgtC[1],TgtC[2],TgtC[3],TgtC[4]*(1-TGTD[k][4]/HoldTick)}
					sC2(TgtCk)
					if MapStyle==4 and DisplayNo==1 then
						px,py=w/2+TGTD[k][5],h+TGTD[k][6]
					else
						px,py=w/2+TGTD[k][5],h/2+TGTD[k][6]
					end
					if px>HMin and px<HMax and py<VMin and py>VMax then
						dR(px,py,0.5,0.5)
					end
				end
			end
		end
	end
end