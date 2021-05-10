function sysCall_init()
         
    vehicleReference=sim.getObjectHandle('youBot_ref')
    vehicleTarget=sim.getObjectHandle('youBot_vehicleTargetPosition')
    sim.setObjectPosition(vehicleTarget,sim.handle_parent,{0,0,0})
    sim.setObjectOrientation(vehicleTarget,sim.handle_parent,{0,0,0})
    sim.setObjectParent(vehicleTarget,-1,true)
    --Prepare initial values and retrieve handles:
    wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheelJoints[1]=sim.getObjectHandle('rollingJoint_fl')
    wheelJoints[2]=sim.getObjectHandle('rollingJoint_rl')
    wheelJoints[3]=sim.getObjectHandle('rollingJoint_rr')
    wheelJoints[4]=sim.getObjectHandle('rollingJoint_fr')
    previousForwBackVel=0
    previousLeftRightVel=0
    previousRotVel=0
    
end
-- This script takes care of the YouBot mobile platform:
-- The platform will try to follow the position and orientation of the 'youBot_vehicleTargetPosition' object


function sysCall_cleanup() 
 
end 

function sysCall_actuation() 
    relP=sim.getObjectPosition(vehicleTarget,vehicleReference)
    relE=sim.getObjectOrientation(vehicleTarget,vehicleReference)
    pParam=20
    maxV=2
    pParamRot=10
    maxVRot=3
    accelF=0.035
    
    
    forwBackVel=relP[2]*pParam
    leftRightVel=relP[1]*pParam
    v=math.sqrt(forwBackVel*forwBackVel+leftRightVel*leftRightVel)
    if v>maxV then
        forwBackVel=forwBackVel*maxV/v
        leftRightVel=leftRightVel*maxV/v
    end
    rotVel=-relE[3]*pParamRot
    if (math.abs(rotVel)>maxVRot) then
        rotVel=maxVRot*rotVel/math.abs(rotVel)
    end
    
    df=forwBackVel-previousForwBackVel
    ds=leftRightVel-previousLeftRightVel
    dr=rotVel-previousRotVel
    
    if (math.abs(df)>maxV*accelF) then
        df=math.abs(df)*(maxV*accelF)/df
    end
    
    if (math.abs(ds)>maxV*accelF) then
        ds=math.abs(ds)*(maxV*accelF)/ds
    end
    
    if (math.abs(dr)>maxVRot*accelF) then
        dr=math.abs(dr)*(maxVRot*accelF)/dr
    end
    
    
    forwBackVel=previousForwBackVel+df
    leftRightVel=previousLeftRightVel+ds
    rotVel=previousRotVel+dr
    
    sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
    sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
    
    previousForwBackVel=forwBackVel
    previousLeftRightVel=leftRightVel
    previousRotVel=rotVel
end 

