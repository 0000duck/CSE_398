waitToReachVehicleTargetPositionAndOrientation=function()
    repeat
        sim.switchThread() -- don't waste your time waiting!
        p1=sim.getObjectPosition(vehicleTarget,-1)
        vehicleReference=sim.getObjectHandle('youBot_ref')
        p2=sim.getObjectPosition(vehicleReference,-1)
        p={p2[1]-p1[1],p2[2]-p1[2]}
        pError=math.sqrt(p[1]*p[1]+p[2]*p[2])
        oError=math.abs(sim.getObjectOrientation(vehicleReference,vehicleTarget)[3])
    until (pError<0.001)and(oError<0.1*math.pi/180) 
end


displayInfo=function(txt)
    if dlgHandle then
        sim.endDialog(dlgHandle)
    end
    dlgHandle=nil
    if txt and #txt>0 then
        dlgHandle=sim.displayDialog('info',txt,sim.dlgstyle_message,false)
        sim.switchThread()
    end
end

getBoxAdjustedMatrixAndFacingAngle=function(boxHandle)
    p2=sim.getObjectPosition(boxHandle,-1)
    vehicleReference=sim.getObjectHandle('youBot_ref')
    p1=sim.getObjectPosition(vehicleReference,-1)
    p={p2[1]-p1[1],p2[2]-p1[2],p2[3]-p1[3]}
    pl=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
    p[1]=p[1]/pl
    p[2]=p[2]/pl
    p[3]=p[3]/pl
    local m=sim.getObjectMatrix(boxHandle,-1)
    matchingScore=0
    for i=1,3,1 do
        v={m[0+i],m[4+i],m[8+i]}
        score=v[1]*p[1]+v[2]*p[2]+v[3]*p[3]
        if (math.abs(score)>matchingScore) then
            s=1
            if (score<0) then s=-1 end
            matchingScore=math.abs(score)
            bestMatch={v[1]*s,v[2]*s,v[3]*s}
        end
    end
    angle=math.atan2(bestMatch[2],bestMatch[1])
    m=sim.buildMatrix(p2,{0,0,angle})
    return m, angle-math.pi/2
end

getMatrixShiftedAlongZ=function(matrix,localZShift)
    -- Returns a pose or matrix shifted by localZShift along the matrix's z-axis
    local m={}
    for i=1,12,1 do
        m[i]=matrix[i]
    end
    m[4]=m[4]+m[3]*localZShift
    m[8]=m[8]+m[7]*localZShift
    m[12]=m[12]+m[11]*localZShift
    return m
end

forbidThreadSwitches=function(forbid)
    -- Allows or forbids automatic thread switches.
    -- This can be important for threaded scripts. For instance,
    -- you do not want a switch to happen while you have temporarily
    -- modified the robot configuration, since you would then see
    -- that change in the scene display.
    if forbid then
        forbidLevel=forbidLevel+1
        if forbidLevel==1 then
            sim.setThreadAutomaticSwitch(false)
        end
    else
        forbidLevel=forbidLevel-1
        if forbidLevel==0 then
            sim.setThreadAutomaticSwitch(true)
        end
    end
end

findCollisionFreeConfigAndCheckApproach=function(matrix)
    -- Here we search for a robot configuration..
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    -- 3. ..that does not collide and that can perform the IK linear approach
    sim.setObjectMatrix(ikTarget,-1,matrix)
    -- Here we check point 1 & 2:
    local c=sim.getConfigForTipPose(ikGroup,jh,0.65,10,nil,collisionPairs)
    if c then
        -- Here we check point 3:
        local m=getMatrixShiftedAlongZ(matrix,ikShift)
        local path=generateIkPath(c,m,ikSteps)
        if path==nil then
            c=nil
        end
    end
    return c
end

findSeveralCollisionFreeConfigsAndCheckApproach=function(matrix,trialCnt,maxConfigs)
    -- Here we search for several robot configurations...
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    -- 3. ..that does not collide and that can perform the IK linear approach
    forbidThreadSwitches(true)
    sim.setObjectMatrix(ikTarget,-1,matrix)
    local cc=getConfig()
    local cs={}
    local l={}
    for i=1,trialCnt,1 do
        local c=findCollisionFreeConfigAndCheckApproach(matrix)
        if c then
            local dist=getConfigConfigDistance(cc,c)
            local p=0
            local same=false
            for j=1,#l,1 do
                if math.abs(l[j]-dist)<0.001 then
                    -- we might have the exact same config. Avoid that
                    same=true
                    for k=1,#jh,1 do
                        if math.abs(cs[j][k]-c[k])>0.01 then
                            same=false
                            break
                        end
                    end
                end
                if same then
                    break
                end
            end
            if not same then
                cs[#cs+1]=c
                l[#l+1]=dist
            end
        end
        if #l>=maxConfigs then
            break
        end
    end
    forbidThreadSwitches(false)
    if #cs==0 then
        cs=nil
    end
    return cs
end

getConfig=function()
    -- Returns the current robot configuration
    local config={}
    for i=1,#jh,1 do
        config[i]=sim.getJointPosition(jh[i])
    end
    return config
end

setConfig=function(config)
    -- Applies the specified configuration to the robot
    if config then
        for i=1,#jh,1 do
            sim.setJointPosition(jh[i],config[i])
        end
    end
end

getConfigConfigDistance=function(config1,config2)
    -- Returns the distance (in configuration space) between two configurations
    local d=0
    for i=1,#jh,1 do
        local dx=(config1[i]-config2[i])*metric[i]
        d=d+dx*dx
    end
    return math.sqrt(d)
end

getPathLength=function(path)
    -- Returns the length of the path in configuration space
    local d=0
    local l=#jh
    local pc=#path/l
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5]}
        d=d+getConfigConfigDistance(config1,config2)
    end
    return d
end

followPath=function(path)
    -- Follows the specified path points. Each path point is a robot configuration. Here we don't do any interpolation
    if path then
        local l=#jh
        local pc=#path/l
        for i=1,pc,1 do
            local config={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5]}
            setConfig(config)
            sim.switchThread()
        end
    end
end

findPath=function(startConfig,goalConfigs,cnt)
    -- Here we do path planning between the specified start and goal configurations. We run the search cnt times,
    -- and return the shortest path, and its length
    local task=simOMPL.createTask('task')
    simOMPL.setAlgorithm(task,simOMPL.Algorithm.RRTConnect)
    local j1_space=simOMPL.createStateSpace('j1_space',simOMPL.StateSpaceType.joint_position,jh[1],{-169*math.pi/180},{169*math.pi/180},1)
    local j2_space=simOMPL.createStateSpace('j2_space',simOMPL.StateSpaceType.joint_position,jh[2],{-65*math.pi/180},{90*math.pi/180},2)
    local j3_space=simOMPL.createStateSpace('j3_space',simOMPL.StateSpaceType.joint_position,jh[3],{-150*math.pi/180},{146*math.pi/180},3)
    local j4_space=simOMPL.createStateSpace('j4_space',simOMPL.StateSpaceType.joint_position,jh[4],{-102.5*math.pi/180},{102.5*math.pi/180},0)
    local j5_space=simOMPL.createStateSpace('j5_space',simOMPL.StateSpaceType.joint_position,jh[5],{-167.5*math.pi/180},{167.5*math.pi/180},0)
    simOMPL.setStateSpace(task,{j1_space,j2_space,j3_space,j4_space,j5_space})
    simOMPL.setCollisionPairs(task,collisionPairs)
    simOMPL.setStartState(task,startConfig)
    simOMPL.setGoalState(task,goalConfigs[1])
    for i=2,#goalConfigs,1 do
        simOMPL.addGoalState(task,goalConfigs[i])
    end
    local path=nil
    local l=999999999999
    forbidThreadSwitches(true)
    for i=1,cnt,1 do
        local res,_path=simOMPL.compute(task,4,-1,300)
        if res and _path then
            local _l=getPathLength(_path)
            if _l<l then
                l=_l
                path=_path
            end
        end
    end
    forbidThreadSwitches(false)
    simOMPL.destroyTask(task)
    return path,l
end

findShortestPath=function(startConfig,goalConfigs,searchCntPerGoalConfig)
    -- This function will search for several paths between the specified start configuration,
    -- and several of the specified goal configurations. The shortest path will be returned
    forbidThreadSwitches(true)
    local thePath=findPath(startConfig,goalConfigs,searchCntPerGoalConfig)
    forbidThreadSwitches(false)
    return thePath
end

generateIkPath=function(startConfig,goalPose,steps)
    -- Generates (if possible) a linear, collision free path between a robot config and a target pose
    forbidThreadSwitches(true)
    local currentConfig=getConfig()
    setConfig(startConfig)
    sim.setObjectMatrix(ikTarget,-1,goalPose)
    local c=sim.generateIkPath(ikGroup,jh,steps,collisionPairs)
    setConfig(currentConfig)
    forbidThreadSwitches(false)
    return c
end

getReversedPath=function(path)
    -- This function will simply reverse a path
    local retPath={}
    local ptCnt=#path/#jh
    for i=ptCnt,1,-1 do
        for j=1,#jh,1 do
            retPath[#retPath+1]=path[(i-1)*#jh+j]
        end
    end
    return retPath
end

function sysCall_threadmain()
    -- Initialization phase:
    jh={-1,-1,-1,-1,-1}
    for i=1,5,1 do
        jh[i]=sim.getObjectHandle('youBotArmJoint'..(i-1))
    end
    ikGroup=sim.getIkGroupHandle('youBotDamped_group')
    ikTarget=sim.getObjectHandle('youBot_positionTarget')
    collisionPairs={sim.getCollectionHandle('manipulator'),sim.getCollectionHandle('environment')}
    target1=sim.getObjectHandle('testTarget1')
    target2=sim.getObjectHandle('testTarget2')
    target3=sim.getObjectHandle('testTarget3')
    target4=sim.getObjectHandle('testTarget4')
    target5=sim.getObjectHandle('testTarget5')
    target6=sim.getObjectHandle('testTarget6')
    target7=sim.getObjectHandle('testTarget7')
    box1=sim.getObjectHandle('Cuboid1')
    box2=sim.getObjectHandle('Cuboid2')
    box3=sim.getObjectHandle('Cuboid3')
    box4=sim.getObjectHandle('Cuboid4')
    box5=sim.getObjectHandle('Cuboid5')
    box6=sim.getObjectHandle('Cuboid6')
    
    approachDirectionObstacle=sim.getObjectHandle('approachDirectionObstacle')
    metric={0.5,1,1,0.5,1}
    --{-170*math.pi/180},{170*math.pi/180},1)
    --{-90*math.pi/180},{90*math.pi/180},2)
    --{-140*math.pi/180},{140*math.pi/180},3)
    --{-150*math.pi/180},{150*math.pi/180},0)
    forbidLevel=0
    ikShift=0.1
    ikSteps=50
    dist1=0.5
    
    og_config=getConfig()

    -- Main loop:
    local allTargets={target1,target2,target3,target4,target5,target6,target7}
    local allBoxes={box1,box2,box3,box4,box5,box6,target7}
    local targetIndex=1
    while true do
        -- This is the main loop. We move from one target to the next
        local theTarget=allTargets[targetIndex]
        local theBox=allBoxes[targetIndex]
        targetIndex=targetIndex+1
        if targetIndex>7 then
            targetIndex=1
        end
        
        m_move,angle=getBoxAdjustedMatrixAndFacingAngle(theBox)
        
        vehicleTarget=sim.getObjectHandle('youBot_vehicleTargetPosition')
        sim.setObjectPosition(vehicleTarget,-1,{m_move[4]-m_move[1]*dist1,m_move[8]-m_move[5]*dist1,0})
        sim.setObjectOrientation(vehicleTarget,-1,{0,0,angle})
        -- m is the transformation matrix or pose of the current target:
        waitToReachVehicleTargetPositionAndOrientation()
        local m=sim.getObjectMatrix(theTarget,-1)

        -- Compute a pose that is shifted by ikDist along the Z-axis of pose m,
        -- so that we have a final approach that is linear along target axis Z:
        m=getMatrixShiftedAlongZ(m,-ikShift)

        -- Find several configs for pose m, and order them according to the
        -- distance to current configuration (smaller distance is better).
        -- In following function we also check for collisions and whether the
        -- final IK approach is feasable:
        --displayInfo('searching for a maximum of 60 valid goal configurations...')
        local c=findSeveralCollisionFreeConfigsAndCheckApproach(m,300,60)
        while c == nil do
            c=findSeveralCollisionFreeConfigsAndCheckApproach(m,300,60)
        end
        if c ~= nil then
            -- Search a path from current config to a goal config. For each goal
            -- config, search 6 times a path and keep the shortest.
            -- Do this for the first 3 configs returned by findCollisionFreeConfigs.
            -- Since we do not want an approach along the negative Z axis, we place
            -- an artificial obstacle into the scene (the blue orthogon):
            local initialApproachDirectionObstaclePose=sim.getObjectMatrix(approachDirectionObstacle,-1)
            sim.setObjectPosition(approachDirectionObstacle,theTarget,{0,0,-ikShift+0.01})
            sim.setObjectOrientation(approachDirectionObstacle,theTarget,{0,0,0})
            sim.switchThread() -- in order see the change before next operation locks
            --local txt='Found '..#c..' different goal configurations for the desired goal pose.'
            --txt=txt..'&&nNow searching the shortest path of 6 searches...'
            --displayInfo(txt)
            local straight_path=findShortestPath(getConfig(),c,6)
            --displayInfo(nil)

            sim.setObjectMatrix(approachDirectionObstacle,-1,initialApproachDirectionObstaclePose)

            -- Follow the path:
            followPath(straight_path)

            -- For the final approach, the target is the original target pose:
            m=sim.getObjectMatrix(theTarget,-1)

            -- Compute a straight-line path from current config to pose m:
            local path=generateIkPath(getConfig(),m,ikSteps)

            -- Follow the path:
            followPath(path)
            
            sim.setModelProperty(theBox, sim.modelproperty_not_visible)

            -- Generate a reversed path in order to move back:
            if (path ~= nil) then
                path=getReversedPath(path)

                -- Follow the path:
                followPath(path)
            end
            -- Go back to start position
            path = getReversedPath(straight_path)
            followPath(path)
        end
    end
end
