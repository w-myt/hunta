from abaqus import *
from abaqusConstants import *
from caeModules import *
from driverUtils import executeOnCaeStartup
import __main__
import numpy as np 
executeOnCaeStartup()
session.journalOptions.setValues(replayGeometry=INDEX,recoverGeometry=INDEX) 

def model_building(keyword_geometry, keyword_Density, keyword_Emodul, keyword_pratio, keyword_friction, keyword_fxy, keyword_fy, keyword_mxy, keyword_my):
    
    Mdb()

    # create material 
    mdb.models['Model-1'].Material(name='concrete')
    mdb.models['Model-1'].materials['concrete'].Density(table=((keyword_Density, ), ))
    mdb.models['Model-1'].materials['concrete'].Elastic(table=((keyword_Emodul, keyword_pratio), ))

    # create section 
    mdb.models['Model-1'].HomogeneousSolidSection(name='concrete', material='concrete', 
                                                thickness=None)
    
    rows = keyword_geometry.split(';')
    height = []
    diameter = []
    thickness = []
    i = 0
    for row in rows:
        if row:
            row_data = [float(x) for x in row.split(',')]
            height.append([row_data[0],row_data[1]])
            diameter.append([row_data[2],row_data[3]])
            thickness.append(float(row_data[4]))

    ### begin the loop
    for index in range(len(height)):
        # draw the sketch 
        s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
            sheetSize=10000.0)
        g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
        s.setPrimaryObject(option=STANDALONE)
        s.ConstructionLine(point1=(0.0, -50000.0), point2=(0.0, 50000.0))
        s.FixedConstraint(entity=g[2])

        s.Line(point1=(diameter[index][0]-thickness[index], height[index][0]), point2=(diameter[index][0], height[index][0]))
        s.Line(point1=(diameter[index][0], height[index][0]), point2=(diameter[index][1], height[index][1]))
        s.Line(point1=(diameter[index][1], height[index][1]), point2=(diameter[index][1]-thickness[index], height[index][1]))
        s.Line(point1=(diameter[index][1]-thickness[index], height[index][1]), point2=(diameter[index][0]-thickness[index], height[index][0]))

        # Revolution 
        name_temp = 'S' + str(index+1)
        p = mdb.models['Model-1'].Part(name=name_temp, dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p = mdb.models['Model-1'].parts[name_temp]
        p.BaseSolidRevolve(sketch=s, angle=120.0, flipRevolveDirection=ON)
        del mdb.models['Model-1'].sketches['__profile__']

        # assign the section 
        c = p.cells
        cells = c[0:1]
        region = regionToolset.Region(cells=cells)
        p.SectionAssignment(region=region, sectionName='concrete', offset=0.0, 
                            offsetType=MIDDLE_SURFACE, offsetField='', 
                            thicknessAssignment=FROM_SECTION)
        
        # generate instance 
        a = mdb.models['Model-1'].rootAssembly
        name_temp = name_temp + '-1'
        a.Instance(name=name_temp, part=p, dependent=ON)
        
        # generate mesh
        e = p.edges 
        
        pickedEdges = e[1:2] + e[3:4] + e[5:6] + e[8:9]
        p.seedEdgeByNumber(edges=pickedEdges, number=40, constraint=FINER)
        pickedEdges = e[4:5] + e[6:7] + e[10:12] 
        p.seedEdgeByNumber(edges=pickedEdges, number=20, constraint=FINER)
        pickedEdges = e[0:1] + e[2:3] + e[7:8] + e[9:10]
        p.seedEdgeByNumber(edges=pickedEdges, number=4, constraint=FINER)
        p.generateMesh()
        a.regenerate()
        # multiply the instance
        a.RadialInstancePattern(instanceList=(name_temp, ), point=(0.0, 0.0, 0.0), 
                                axis=(0.0, 1.0, 0.0), number=3, totalAngle=360.0)
        # rotate the instances for 60 degree if it is even layer 
        if index%2 == 1:
            a = mdb.models['Model-1'].rootAssembly
            instance2_name = name_temp + '-rad-2'
            instance3_name = name_temp + '-rad-3'
            a.rotate(instanceList=(name_temp, instance2_name, instance3_name), axisPoint=(0.0, 3000.0, 0.0), 
                    axisDirection=(0.0, -10.0, 0.0), angle=60.0)
        ## end of the loop

    #define contact 
    #: create The interaction property "IntProp-1"
    a = mdb.models['Model-1'].rootAssembly
    mdb.models['Model-1'].ContactProperty('IntProp-1')
    mdb.models['Model-1'].interactionProperties['IntProp-1'].TangentialBehavior(formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, 
                                                                                table=((keyword_friction, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, fraction=0.005, elasticSlipStiffness=None)
    mdb.models['Model-1'].interactionProperties['IntProp-1'].NormalBehavior(pressureOverclosure=HARD, allowSeparation=ON, constraintEnforcementMethod=DEFAULT)
    # define the contact surfaces 
    a = mdb.models['Model-1'].rootAssembly
    instance_keys = a.instances.keys()

    for instance_key in instance_keys:
        allFaces = a.instances[instance_key].faces
        side1Faces1 = allFaces[0:1]
        name_temp = instance_key + '-bottom'
        a.Surface(side1Faces = side1Faces1, name = name_temp)

        side1Faces1 = allFaces[2:3]
        name_temp = instance_key + '-top'
        a.Surface(side1Faces = side1Faces1, name = name_temp)

        allFaces = a.instances[instance_key].faces
        side1Faces1 = allFaces[4:5]
        name_temp = instance_key + '-right'
        a.Surface(side1Faces = side1Faces1, name = name_temp)

        allFaces = a.instances[instance_key].faces
        side1Faces1 = allFaces[5:6]
        name_temp = instance_key + '-left'
        a.Surface(side1Faces = side1Faces1, name = name_temp)

    # define contact within each layer 
    # 核心修复：强制转换为整数，杜绝浮点数/NaN报错
    layer_number = int(len(instance_keys) / 3)

    for i in range(layer_number):
        # 1-3
        a = mdb.models['Model-1'].rootAssembly
        region1_name = 'S'+ str(i+1) + '-1-left'
        region2_name = 'S'+ str(i+1) + '-1-rad-3-right'
        region1 = a.surfaces[region1_name]
        region2 = a.surfaces[region2_name]
        contact_name = 'ctc-'+'S' + str(i+1) + '-1-3'
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial', name= contact_name, main=region1, secondary=region2, sliding=FINITE, thickness=ON,
							 interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
         
        # 1-2
        a = mdb.models['Model-1'].rootAssembly
        region1_name = 'S'+ str(i+1) + '-1-right'
        region2_name = 'S'+ str(i+1) + '-1-rad-2-left'
        contact_name = 'ctc-'+'S' + str(i+1) + '-1-2'
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                         interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
         
        # 2-3 
        a = mdb.models['Model-1'].rootAssembly
        region1_name = 'S'+ str(i+1) + '-1-rad-2-right'
        region2_name = 'S'+ str(i+1) + '-1-rad-3-left'
        contact_name = 'ctc-'+'S' + str(i+1) + '-2-3'
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                         interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
        
    # define contact between layers 
    for i in range(layer_number - 1):

        a = mdb.models['Model-1'].rootAssembly
        region1_name = 'S'+ str(i+1) + '-1-top'
        region2_name = 'S'+ str(i+2) + '-1-bottom'
        contact_name = 'ctc-'+'S' + str(i+1) + '-' + 'S' + str(i+2) + '-2'
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                         interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
        
        a = mdb.models['Model-1'].rootAssembly
        region1_name = 'S'+ str(i+1) + '-1-rad-2-top'
        region2_name = 'S'+ str(i+2) + '-1-rad-2-bottom'
        contact_name = 'ctc-'+'S' + str(i+1) + '-' + 'S' + str(i+2) + '-4'
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                         interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)   
                 
        a = mdb.models['Model-1'].rootAssembly
        region1_name = 'S'+ str(i+1) + '-1-rad-3-top'
        region2_name = 'S'+ str(i+2) + '-1-rad-3-bottom'
        contact_name = 'ctc-'+'S' + str(i+1) + '-' + 'S' + str(i+2) + '-5'
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                         interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
        
        if i%2 == 0: 

            a = mdb.models['Model-1'].rootAssembly
            region1_name = 'S'+ str(i+1) + '-1-top'
            region2_name = 'S'+ str(i+2) + '-1-rad-2-bottom'
            contact_name = 'ctc-'+'S' + str(i+1) + '-' + 'S' + str(i+2) + '-1'
            mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                         interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
            
            a = mdb.models['Model-1'].rootAssembly
            region1_name = 'S'+ str(i+1) + '-1-rad-2-top'
            region2_name = 'S'+ str(i+2) + '-1-rad-3-bottom'
            contact_name = 'ctc-'+'S' + str(i+1) + '-' + 'S' + str(i+2) + '-3'
            mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                            interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
        
            a = mdb.models['Model-1'].rootAssembly
            region1_name = 'S'+ str(i+1) + '-1-rad-3-top'
            region2_name = 'S'+ str(i+2) + '-1-bottom'
            contact_name = 'ctc-'+'S' + str(i+1) + '-' + 'S' + str(i+2) + '-6'
            mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                            interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
        
        else:

            a = mdb.models['Model-1'].rootAssembly
            region1_name = 'S'+ str(i+1) + '-1-top'
            region2_name = 'S'+ str(i+2) + '-1-rad-3-bottom'
            contact_name = 'ctc-'+'S' + str(i+1) + '-' + 'S' + str(i+2) + '-1'
            mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                         interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
            
            a = mdb.models['Model-1'].rootAssembly
            region1_name = 'S'+ str(i+1) + '-1-rad-2-top'
            region2_name = 'S'+ str(i+2) + '-1-bottom'
            contact_name = 'ctc-'+'S' + str(i+1) + '-' + 'S' + str(i+2) + '-3'
            mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                            interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
        
            a = mdb.models['Model-1'].rootAssembly
            region1_name = 'S'+ str(i+1) + '-1-rad-3-top'
            region2_name = 'S'+ str(i+2) + '-1-rad-2-bottom'
            contact_name = 'ctc-'+'S' + str(i+1) + '-' + 'S' + str(i+2) + '-6'
            mdb.models['Model-1'].SurfaceToSurfaceContactStd(createStepName='Initial',  name= contact_name, main=a.surfaces[region1_name], secondary=a.surfaces[region2_name], sliding=FINITE, thickness=ON, 
                                                            interactionProperty='IntProp-1', adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, clearanceRegion=None)
    
    # define load 
    # define load introduction point 
    a = mdb.models['Model-1'].rootAssembly
    a.ReferencePoint(point=(0.0, height[-1][-1], 0.0))

    # define coupling 
    a = mdb.models['Model-1'].rootAssembly
    r1 = a.referencePoints
    r1_key = a.referencePoints.keys()
    refPoints1=(r1[r1_key[0]], )
    region1=regionToolset.Region(referencePoints=refPoints1)
    a = mdb.models['Model-1'].rootAssembly
    # 修复：layer_number已为整数，实例名称无小数，解决KeyError
    f1_name = 'S' + str(layer_number) + '-1-rad-3'
    e1 = a.instances[f1_name].edges
    edges1 = e1[8:9]
    f2_name = 'S' + str(layer_number) + '-1-rad-2'
    e2 = a.instances[f2_name].edges
    edges2 = e2[8:9]
    f3_name = 'S' + str(layer_number) + '-1'
    e3 = a.instances[f3_name].edges
    edges3 = e3[8:9]
    region2=regionToolset.Region(edges=edges1+edges2+edges3)
    mdb.models['Model-1'].Coupling(name='load_point', controlPoint=region1, surface=region2, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC, localCsys=None, u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)

    #define load step 
    mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial', maxNumInc=10000, initialInc=0.001, minInc=1e-07, maxInc=0.01, nlgeom=ON)

    #define load components 
    # fx
    a = mdb.models['Model-1'].rootAssembly
    mdb.models['Model-1'].ConcentratedForce(name='Fx', createStepName='Step-1', region=region1, cf1=keyword_fxy, distributionType=UNIFORM, field='', localCsys=None)

    # fy
    a = mdb.models['Model-1'].rootAssembly
    mdb.models['Model-1'].ConcentratedForce(name='Fy', createStepName='Step-1', region=region1, cf2=keyword_fy, distributionType=UNIFORM, field='', localCsys=None)

    # Mxy
    a = mdb.models['Model-1'].rootAssembly
    mdb.models['Model-1'].Moment(name='Mxy', createStepName='Step-1', region=region1, cm3=keyword_mxy, distributionType=UNIFORM, field='', localCsys=None)

    # My
    a = mdb.models['Model-1'].rootAssembly
    mdb.models['Model-1'].Moment(name='My', createStepName='Step-1', region=region1, cm2=keyword_my, distributionType=UNIFORM, field='', localCsys=None)

    # define boundary conditons 
    a = mdb.models['Model-1'].rootAssembly

    f1 = a.instances['S1-1-rad-3'].faces
    faces1 = f1[0:1]
    f2 = a.instances['S1-1-rad-2'].faces
    faces2 = f2[0:1]
    f3 = a.instances[ 'S1-1'].faces
    faces3 = f3[0:1]
    region = regionToolset.Region(faces=faces1+faces2+faces3)
    mdb.models['Model-1'].DisplacementBC(name='BC-1', createStepName='Step-1', region=region, u1=0.0, u2=0.0, u3=0.0, ur1=0.0, ur2=0.0, ur3=0.0, amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                         localCsys=None)

    # solve 【已按第一种方案修改】
    mdb.Job(name='Job-1', model='Model-1', description='', type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, explicitPrecision=SINGLE, 
            nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', scratch='', resultsFormat=ODB, numThreadsPerMpiProcess=1, multiprocessingMode=DEFAULT, 
            numCpus=1, numDomains=1, numGPUs=0)
    # 关闭自动提交，避免系统找不到文件
    # mdb.jobs['Job-1'].submit(consistencyChecking=OFF)
    # 生成INP文件，手动在Abaqus中提交求解
    mdb.jobs['Job-1'].writeInput()
