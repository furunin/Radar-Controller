--config
MinRange=10
MargeDist=10
MaxTGT=32
HoldTick=180

iB=input.getBool
iN=input.getNumber
oB=output.setBool
oN=output.setNumber
m=math
pi=m.pi
pi2=2*pi
s=m.sin
c=m.cos
t=m.tan
as=m.asin
ac=m.acos
at=m.atan
sc=screen
dT=sc.drawText
dL=sc.drawLine
dC=sc.drawCircle
fo=string.format
tb=table
tI=tb.insert
tR=tb.remove
pN=property.getNumber
pD={pN("GPS X horizontal offset (m)"),pN("GPS Y directional offset (m)"),pN("Altimeter vertical offset (m)")}
delay=pN("Process Delay (tick)")
Marge=property.getBool("Marge close targets")
DAD=pN("Installation Posture")
MaxHist=pN("Smoothing tick")
RType=pN("Radar Type")
if RType==0     then isSR,isTR=true,false
elseif RType==1 then isSR,isTR=true,true
elseif RType==2 then isSR,isTR=false,true
end
if DAD>0 then
	if DAD==2 then fy=pi/2
	elseif DAD==3 then fy=pi
	elseif DAD==4 then fy=-pi/2
	else fy=0
	end
	rx={c(fy),0,-s(fy)}
	ry={0,1,0}
	rz={s(fy),0,c(fy)}
else
	fx=pi/2
	if DAD==-2 then fz=pi/2
	elseif DAD==-3 then fz=0
	elseif DAD==-4 then fz=-pi/2
	else fz=pi
	end
	rx={c(fz),s(fz),0}
	ry={-s(fz)*c(fx),c(fz)*c(fx),s(fx)}
	rz={s(fz)*s(fx),-c(fz)*s(fx),c(fx)}
end
rs={rx,ry,rz}
function distV(V1,V2)
	_=0
	for i=1,3 do
		_=_+(V1[i]-V2[i])^2	
	end
	_=_^0.5
	return _
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
function inv(e1,e2,e3)
	a,b,C,d,e,f,g,h,i=e1[1],e2[1],e3[1],e1[2],e2[2],e3[2],e1[3],e2[3],e3[3]
	det=a*e*i+d*h*C+g*b*f-g*e*C-a*h*f-d*b*i
	E1={(e*i-f*h)/det,(f*g-d*i)/det,(d*h-e*g)/det}
	E2={(C*h-b*i)/det,(a*i-C*g)/det,(b*g-a*h)/det}
	E3={(b*f-C*e)/det,(C*d-a*f)/det,(a*e-b*d)/det}
	return E1,E2,E3,det
end
function PolToOrth(VP)
	VO={}
	VO[1]=VP[1]*c(VP[3])*s(VP[2])
	VO[2]=VP[1]*c(VP[3])*c(VP[2])
	VO[3]=VP[1]*s(VP[3])
	return VO
end
cID=1
function RegisterTGT(tgtV)
	tI(TGT,{tgtV})
	tgtV[6]=cID
	tI(TGTS,tgtV)
	if #TGT>MaxTGT then
		tR(TGT,1)
		tR(TGTS,1)
	end
	cID=cID+1
end
PTG={}
TGT={}
TGTS={}

function onTick()
	oB(31,isSR)
	oB(32,isTR)
	
	PO={iN(4),iN(8),iN(12)}
	QO={iN(16)*pi2,iN(20)*pi2,iN(24)*pi2,iN(28)*pi2}
	es=AngleToBasis(QO)
	cb=iN(32)*pi2
	--Input and transform
	for i=1,8 do
		det=iB(i)
		dist,azim,elev=iN(i*4-3),iN(i*4-2)*pi2,iN(i*4-1)*pi2
		if det and dist>MinRange then
			PTP={dist,azim,elev}
			PTL=PolToOrth(PTP)
			for j=1,3 do
				PTL[j]=PTL[j]+pD[j]
			end
			PTG[i]=Mv(rs,PTL)
			PTG[i]=Mv(es,PTL)
			for j=1,3 do
				PTG[i][j]=PTG[i][j]+PO[j]
			end
			PTG[i][4]=delay
			PTG[i][5]=true
		else
			PTG[i]={0,0,0,-1,false,-1}
		end
	end
	--Marge close targets
	if Marge then
		for i=8,2,-1 do
		for k=i-1,1,-1 do
			if PTG[i][5] and PTG[k][5] and distV(PTG[i],PTG[k])<MargeDist then
				for j=1,3 do
					PTG[k][j]=(PTG[k][j]+PTG[i][j])/2
				end
				PTG[i]={0,0,0,-1,false,-1}
				break
			end
		end
		end
	end
	--Target identification and smoothing
	for i=1,8 do
		if PTG[i][5] then
			if #TGT>0 then
				dist,km=distV(PTG[i],TGT[1][1]),1
				for k=1,#TGT do
					distk=distV(PTG[i],TGT[k][1])
					if distk<dist then
						dist=distk
						km=k
					end
				end
				if dist<MargeDist then
					tI(TGT[km],1,PTG[i])
					if #TGT[km]>MaxHist then tR(TGT[km]) end
					for j=1,3 do
						cmin=TGT[km][1][j]
						cmax=cmin
						for l=1,#TGT[km] do
							cl=TGT[km][l][j]
							if cl<cmin then
								cmin=cl
							elseif cl>cmax then
								cmax=cl
							end
						end
						TGTS[km][j]=(cmin+cmax)/2
					end
					TGTS[km][4]=PTG[i][4]
					PTG[i][6]=TGTS[km][6]
				else
					RegisterTGT(PTG[i])
					PTG[i][6]=cID-1
				end
			else
				RegisterTGT(PTG[i])
				PTG[i][6]=cID-1
			end
		end
	end
	--Target data number check
	if #TGT>0 then
		for k=#TGT,1,-1 do
			if TGTS[k][4]>MaxHist then
				tR(TGT,k)
				tR(TGTS,k)
			end
		end
	end
	--Taret data output
	for i=1,8 do
		if PTG[i][5] then
			ki=-1
			for k=1,#TGT do
				if PTG[i][6]==TGTS[k][6] then
					ki=k
					break
				end
			end
			OUT={}
			for j=1,4 do
				OUT[j]=TGTS[ki][j]
			end
			OUT[5]=true
		else
			OUT={0,0,0,-1,false}
		end
		for j=1,4 do
			oN(i*4-(4-j),OUT[j])
		end
		oB(i,OUT[5])
	end
	--Target data tick update (only latest data)
	if #TGT>0 then
		for k=1,#TGT do
			TGTS[k][4]=TGTS[k][4]+1
		end
	end

end

function onDraw()
	dT(1,1,"---X ".."---Y ".."---Z ")
	for i=1,8 do
		dT(1+0*25,i*8,fo("%4.0f",PTG[i][1]))
		dT(1+1*25,i*8,fo("%4.0f",PTG[i][2]))
		dT(1+2*25,i*8,fo("%4.0f",PTG[i][3]))
	end
end