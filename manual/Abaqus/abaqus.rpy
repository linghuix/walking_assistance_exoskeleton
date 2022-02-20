# -*- coding: mbcs -*-
#
# Abaqus/CAE Release 2016 replay file
# Internal Version: 2015_09_25-04.31.09 126547
# Run by XLH on Fri Nov 13 20:40:46 2020
#

# from driverUtils import executeOnCaeGraphicsStartup
# executeOnCaeGraphicsStartup()
#: Executing "onCaeGraphicsStartup()" in the site directory ...
from abaqus import *
from abaqusConstants import *
session.Viewport(name='Viewport: 1', origin=(0.0, 0.0), width=329.522888183594, 
    height=243.911117553711)
session.viewports['Viewport: 1'].makeCurrent()
session.viewports['Viewport: 1'].maximize()
from caeModules import *
from driverUtils import executeOnCaeStartup
executeOnCaeStartup()
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=ON)
openMdb(
    pathName='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/Level1.cae')
#: The model database "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\Level1.cae" has been opened.
session.viewports['Viewport: 1'].setValues(displayedObject=None)
p = mdb.models['fracture cut'].parts['LEVEL1']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
#--- Recover file: 'Level1.rec' ---
# -*- coding: mbcs -*- 
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
mdb.models['fracture cut'].steps['Step-1'].setValues(initialInc=0.01, 
    maxInc=1.0, maxNumInc=1000, minInc=1e-05, timePeriod=1.0)
mdb.jobs['drag-200N'].submit(consistencyChecking=OFF)
mdb.jobs['drag-200N']._Message(STARTED, {'phase': BATCHPRE_PHASE, 
    'clientHost': 'DESKTOP-GDPPV8K', 'handle': 0, 'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(WARNING, {'phase': BATCHPRE_PHASE, 
    'message': '29 elements are distorted. Either the isoparametric angles are out of the suggested limits or the triangular or tetrahedral quality measure is bad. The elements have been identified in element set WarnElemDistorted.', 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(WARNING, {'phase': BATCHPRE_PHASE, 
    'message': 'OUTPUT REQUEST V IS NOT AVAILABLE FOR THIS TYPE OF ANALYSIS', 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(WARNING, {'phase': BATCHPRE_PHASE, 
    'message': 'OUTPUT REQUEST VT IS NOT AVAILABLE FOR THIS TYPE OF ANALYSIS', 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(WARNING, {'phase': BATCHPRE_PHASE, 
    'message': 'DEGREE OF FREEDOM 4 IS NOT ACTIVE IN THIS MODEL AND CANNOT BE RESTRAINED.', 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(WARNING, {'phase': BATCHPRE_PHASE, 
    'message': 'DEGREE OF FREEDOM 5 IS NOT ACTIVE IN THIS MODEL AND CANNOT BE RESTRAINED.', 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(WARNING, {'phase': BATCHPRE_PHASE, 
    'message': 'DEGREE OF FREEDOM 6 IS NOT ACTIVE IN THIS MODEL AND CANNOT BE RESTRAINED.', 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(ODB_FILE, {'phase': BATCHPRE_PHASE, 
    'file': 'E:\\101-WorkShop\\simulation\\solidwork\\fractal cut\\abaqus-simulation\\multiLevel_1\\drag-200N\\drag-200N.odb', 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(COMPLETED, {'phase': BATCHPRE_PHASE, 
    'message': 'Analysis phase complete', 'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(STARTED, {'phase': STANDARD_PHASE, 
    'clientHost': 'DESKTOP-GDPPV8K', 'handle': 13808, 'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(STEP, {'phase': STANDARD_PHASE, 'stepId': 1, 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(ODB_FRAME, {'phase': STANDARD_PHASE, 'step': 0, 
    'frame': 0, 'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(MEMORY_ESTIMATE, {'phase': STANDARD_PHASE, 
    'jobName': 'drag-200N', 'memory': 4961.0})
mdb.jobs['drag-200N']._Message(WARNING, {'phase': STANDARD_PHASE, 
    'message': 'EXCESSIVE DISTORTION AT A TOTAL OF 96966 INTEGRATION POINTS IN SOLID (CONTINUUM) ELEMENTS', 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(STATUS, {'totalTime': 0.0, 'attempts': ' 1U', 
    'timeIncrement': 0.01, 'increment': 1, 'stepTime': 0.0, 'step': 1, 
    'jobName': 'drag-200N', 'severe': 0, 'iterations': 4, 
    'phase': STANDARD_PHASE, 'equilibrium': 4})
mdb.jobs['drag-200N']._Message(STATUS, {'totalTime': 0.0, 'attempts': ' 2U', 
    'timeIncrement': 0.0025, 'increment': 1, 'stepTime': 0.0, 'step': 1, 
    'jobName': 'drag-200N', 'severe': 0, 'iterations': 7, 
    'phase': STANDARD_PHASE, 'equilibrium': 7})
mdb.jobs['drag-200N']._Message(ODB_FRAME, {'phase': STANDARD_PHASE, 'step': 0, 
    'frame': 1, 'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(STATUS, {'totalTime': 0.000625, 'attempts': 3, 
    'timeIncrement': 0.000625, 'increment': 1, 'stepTime': 0.000625, 'step': 1, 
    'jobName': 'drag-200N', 'severe': 0, 'iterations': 6, 
    'phase': STANDARD_PHASE, 'equilibrium': 6})
mdb.jobs['drag-200N']._Message(ODB_FRAME, {'phase': STANDARD_PHASE, 'step': 0, 
    'frame': 2, 'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(STATUS, {'totalTime': 0.00125, 'attempts': 1, 
    'timeIncrement': 0.000625, 'increment': 2, 'stepTime': 0.00125, 'step': 1, 
    'jobName': 'drag-200N', 'severe': 0, 'iterations': 6, 
    'phase': STANDARD_PHASE, 'equilibrium': 6})
mdb.jobs['drag-200N']._Message(ODB_FRAME, {'phase': STANDARD_PHASE, 'step': 0, 
    'frame': 3, 'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(STATUS, {'totalTime': 0.001875, 'attempts': 1, 
    'timeIncrement': 0.000625, 'increment': 3, 'stepTime': 0.001875, 'step': 1, 
    'jobName': 'drag-200N', 'severe': 0, 'iterations': 3, 
    'phase': STANDARD_PHASE, 'equilibrium': 3})
mdb.jobs['drag-200N']._Message(ODB_FRAME, {'phase': STANDARD_PHASE, 'step': 0, 
    'frame': 4, 'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(STATUS, {'totalTime': 0.0025, 'attempts': 1, 
    'timeIncrement': 0.000625, 'increment': 4, 'stepTime': 0.0025, 'step': 1, 
    'jobName': 'drag-200N', 'severe': 0, 'iterations': 3, 
    'phase': STANDARD_PHASE, 'equilibrium': 3})
mdb.jobs['drag-200N']._Message(ERROR, {'phase': STANDARD_PHASE, 
    'message': 'Process terminated by external request (SIGTERM or SIGINT received).', 
    'jobName': 'drag-200N'})
mdb.jobs['drag-200N']._Message(INTERRUPTED, {'phase': STANDARD_PHASE, 
    'message': 'Analysis interrupted by external signal', 
    'jobName': 'drag-200N'})
#--- End of Recover file ------
a = mdb.models['fracture cut'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON, optimizationTasks=OFF, 
    geometricRestrictions=OFF, stopConditions=OFF)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Step-1')
import os
os.chdir(
    r"E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\multiLevel_1\drag-200N")
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF, adaptiveMeshConstraints=ON)
mdb.models['fracture cut'].steps['Step-1'].setValues(timePeriod=0.05, 
    minInc=5e-07, maxInc=0.05)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=OFF)
mdb.jobs['drag-200N'].submit(consistencyChecking=OFF)
#: The job input file "drag-200N.inp" has been submitted for analysis.
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=ON)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=OFF)
#: Job drag-200N: Analysis Input File Processor completed successfully.
#: Error in job drag-200N: Time increment required is less than the minimum specified
#: Error in job drag-200N: THE ANALYSIS HAS BEEN TERMINATED DUE TO PREVIOUS ERRORS. ALL OUTPUT REQUESTS HAVE BEEN WRITTEN FOR THE LAST CONVERGED INCREMENT.
#: Job drag-200N: Abaqus/Standard aborted due to errors.
#: Error in job drag-200N: Abaqus/Standard Analysis exited with an error - Please see the  message file for possible error messages if the file exists.
#: Job drag-200N aborted due to errors.
session.viewports['Viewport: 1'].view.setValues(nearPlane=938.321, 
    farPlane=1570.58, width=758.593, height=424.441, viewOffsetX=-8.5582, 
    viewOffsetY=36.8765)
mdb.save()
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\Level1.cae".
Mdb()
#: A new model database has been created.
#: The model "Model-1" has been created.
session.viewports['Viewport: 1'].setValues(displayedObject=None)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.saveAs(
    pathName='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/single')
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\single.cae".
step = mdb.openStep(
    'E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/multiLevel_1/LEVEL1.STEP', 
    scaleFromFile=OFF)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-1', geometryFile=step, 
    combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-2', geometryFile=step, 
    bodyNum=2, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-3', geometryFile=step, 
    bodyNum=3, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-4', geometryFile=step, 
    bodyNum=4, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-5', geometryFile=step, 
    bodyNum=5, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-6', geometryFile=step, 
    bodyNum=6, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-7', geometryFile=step, 
    bodyNum=7, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-8', geometryFile=step, 
    bodyNum=8, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-9', geometryFile=step, 
    bodyNum=9, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-10', geometryFile=step, 
    bodyNum=10, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-11', geometryFile=step, 
    bodyNum=11, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-12', geometryFile=step, 
    bodyNum=12, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-13', geometryFile=step, 
    bodyNum=13, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-14', geometryFile=step, 
    bodyNum=14, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-15', geometryFile=step, 
    bodyNum=15, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
mdb.models['Model-1'].PartFromGeometryFile(name='LEVEL1-16', geometryFile=step, 
    bodyNum=16, combine=False, dimensionality=THREE_D, type=DEFORMABLE_BODY)
p = mdb.models['Model-1'].parts['LEVEL1-16']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
p1 = mdb.models['Model-1'].parts['LEVEL1-16']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
session.viewports['Viewport: 1'].view.setValues(nearPlane=236.61, 
    farPlane=390.963, width=165.899, height=92.8222, viewOffsetX=-3.47015, 
    viewOffsetY=0.169626)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=ON)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=OFF)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(interactions=ON, 
    constraints=ON, connectors=ON, engineeringFeatures=ON)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(interactions=OFF, 
    constraints=OFF, connectors=OFF, engineeringFeatures=OFF)
session.viewports['Viewport: 1'].view.setValues(nearPlane=3.97852, 
    farPlane=5.02148, width=0.276258, height=0.154569, viewOffsetX=-0.737111, 
    viewOffsetY=-0.0238582)
session.viewports['Viewport: 1'].view.fitView()
a = mdb.models['Model-1'].rootAssembly
a.DatumCsysByDefault(CARTESIAN)
p = mdb.models['Model-1'].parts['LEVEL1-1']
a.Instance(name='LEVEL1-1-1', part=p, dependent=ON)
session.viewports['Viewport: 1'].view.setValues(nearPlane=421.634, 
    farPlane=724.376, width=479.978, height=268.553, viewOffsetX=16.5733, 
    viewOffsetY=-3.53648)
a = mdb.models['Model-1'].rootAssembly
del a.features['LEVEL1-1-1']
p1 = mdb.models['Model-1'].parts['LEVEL1-4']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p1 = mdb.models['Model-1'].parts['LEVEL1-10']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
session.viewports['Viewport: 1'].view.setValues(nearPlane=204.191, 
    farPlane=423.381, width=540.536, height=302.436, viewOffsetX=-55.3565, 
    viewOffsetY=57.7659)
p1 = mdb.models['Model-1'].parts['LEVEL1-1']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
session.viewports['Viewport: 1'].view.setValues(nearPlane=231.597, 
    farPlane=395.976, width=250.408, height=140.106, viewOffsetX=2.56626, 
    viewOffsetY=1.5965)
p1 = mdb.models['Model-1'].parts['LEVEL1-11']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
session.viewports['Viewport: 1'].view.setValues(nearPlane=221.975, 
    farPlane=405.597, width=373.14, height=208.776, viewOffsetX=-0.266217, 
    viewOffsetY=26.7912)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
a1 = mdb.models['Model-1'].rootAssembly
p = mdb.models['Model-1'].parts['LEVEL1-3']
a1.Instance(name='LEVEL1-3-1', part=p, dependent=ON)
session.viewports['Viewport: 1'].view.setValues(nearPlane=261.246, 
    farPlane=447.412, width=258.829, height=144.817, viewOffsetX=0.469303, 
    viewOffsetY=8.87982)
mdb.save()
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\single.cae".
mdb.save()
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\single.cae".
p1 = mdb.models['Model-1'].parts['LEVEL1-12']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
del mdb.models['Model-1'].parts['LEVEL1-12']
del mdb.models['Model-1'].parts['LEVEL1-1']
del mdb.models['Model-1'].parts['LEVEL1-2']
del mdb.models['Model-1'].parts['LEVEL1-3']
del mdb.models['Model-1'].parts['LEVEL1-4']
del mdb.models['Model-1'].parts['LEVEL1-5']
del mdb.models['Model-1'].parts['LEVEL1-6']
del mdb.models['Model-1'].parts['LEVEL1-7']
del mdb.models['Model-1'].parts['LEVEL1-8']
del mdb.models['Model-1'].parts['LEVEL1-9']
del mdb.models['Model-1'].parts['LEVEL1-10']
del mdb.models['Model-1'].parts['LEVEL1-11']
p = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
#* FeatureError: Regeneration failed
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p1 = mdb.models['Model-1'].parts['LEVEL1-15']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p1 = mdb.models['Model-1'].parts['LEVEL1-15']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p1 = mdb.models['Model-1'].parts['LEVEL1-16']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
session.viewports['Viewport: 1'].view.setValues(nearPlane=220.306, 
    farPlane=407.267, width=393.972, height=220.432, viewOffsetX=12.1424, 
    viewOffsetY=-4.80718)
p1 = mdb.models['Model-1'].parts['LEVEL1-14']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
del mdb.models['Model-1'].parts['LEVEL1-14']
del mdb.models['Model-1'].parts['LEVEL1-15']
del mdb.models['Model-1'].parts['LEVEL1-16']
p = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
a = mdb.models['Model-1'].rootAssembly
del a.features['LEVEL1-3-1']
a1 = mdb.models['Model-1'].rootAssembly
p = mdb.models['Model-1'].parts['LEVEL1-13']
a1.Instance(name='LEVEL1-13-1', part=p, dependent=ON)
session.viewports['Viewport: 1'].view.setValues(nearPlane=625.508, 
    farPlane=958.321, width=294.404, height=164.722, viewOffsetX=-4.54904, 
    viewOffsetY=73.1221)
mdb.save()
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\single.cae".
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=OFF)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
mdb.models['Model-1'].Material(name='Al')
mdb.models['Model-1'].materials['Al'].Density(table=((2700.0, ), ))
mdb.models['Model-1'].materials['Al'].Elastic(table=((600000.0, 0.3), ))
mdb.models['Model-1'].HomogeneousShellSection(name='Section-1', 
    preIntegrate=OFF, material='Al', thicknessType=UNIFORM, thickness=0.1, 
    thicknessField='', idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, 
    thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, 
    integrationRule=SIMPSON, numIntPts=5)
session.viewports['Viewport: 1'].view.setValues(nearPlane=226.74, 
    farPlane=404.361, width=179.921, height=100.668, cameraPosition=(186.056, 
    453.857, 61.004), cameraUpVector=(-0.630358, 0.595751, -0.497725), 
    cameraTarget=(-78.2282, 296.959, -2.2311))
session.viewports['Viewport: 1'].view.setValues(nearPlane=228.603, 
    farPlane=403.469, width=181.4, height=101.495, cameraPosition=(200.024, 
    440.136, 21.1999), cameraUpVector=(-0.646733, 0.602852, -0.467232), 
    cameraTarget=(-78.1501, 296.882, -2.45365))
session.viewports['Viewport: 1'].view.setValues(nearPlane=228.836, 
    farPlane=402.955, width=181.585, height=101.598, cameraPosition=(199.713, 
    437.405, 36.3918), cameraUpVector=(-0.624882, 0.621472, -0.472541), 
    cameraTarget=(-78.1523, 296.863, -2.34552))
session.viewports['Viewport: 1'].view.setValues(nearPlane=231.272, 
    farPlane=400.519, width=143.281, height=80.1674, viewOffsetX=-0.543735, 
    viewOffsetY=1.15415)
session.viewports['Viewport: 1'].view.setValues(nearPlane=232.645, 
    farPlane=397.986, width=144.132, height=80.6433, cameraPosition=(175.781, 
    453.742, 95.0041), cameraUpVector=(-0.598897, 0.618205, -0.509063), 
    cameraTarget=(-78.1903, 297.039, -1.97223), viewOffsetX=-0.546963, 
    viewOffsetY=1.161)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
region = p.Set(cells=cells, name='Set-1')
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF, mesh=ON)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=ON)
session.viewports['Viewport: 1'].view.setValues(nearPlane=254.017, 
    farPlane=373.222, width=157.373, height=88.2903, cameraPosition=(144.17, 
    244.037, 212.311), cameraUpVector=(-0.184095, 0.981172, -0.0584), 
    cameraTarget=(-78.8089, 296.031, -2.25531), viewOffsetX=-0.59721, 
    viewOffsetY=1.26766)
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.seedPart(size=4.8, deviationFactor=0.1, minSizeFactor=0.1)
session.viewports['Viewport: 1'].view.setValues(width=166.691, height=93.5183, 
    viewOffsetX=-1.67201, viewOffsetY=1.36707)
session.viewports['Viewport: 1'].view.setValues(nearPlane=251.079, 
    farPlane=373.448, width=165.481, height=92.8396, cameraPosition=(-267.06, 
    212.996, 236.808), cameraUpVector=(-0.294327, 0.883287, -0.364932), 
    cameraTarget=(-77.8145, 297.07, 1.05524), viewOffsetX=-1.65988, 
    viewOffsetY=1.35715)
session.viewports['Viewport: 1'].view.setValues(nearPlane=283.042, 
    farPlane=341.848, width=186.547, height=104.658, cameraPosition=(-0.821907, 
    285.801, 302.287), cameraUpVector=(-0.318808, 0.919169, -0.23128), 
    cameraTarget=(-78.8834, 296.091, -1.46014), viewOffsetX=-1.87118, 
    viewOffsetY=1.52992)
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.generateMesh()
session.viewports['Viewport: 1'].view.setValues(nearPlane=278.511, 
    farPlane=346.379, width=207.742, height=116.549, viewOffsetX=11.4566, 
    viewOffsetY=3.20243)
session.viewports['Viewport: 1'].view.setValues(nearPlane=239.686, 
    farPlane=369.019, width=178.782, height=100.302, cameraPosition=(-111.352, 
    59.736, 188.854), cameraUpVector=(-0.361428, 0.834775, 0.415356), 
    cameraTarget=(-77.3326, 299.218, -11.0286), viewOffsetX=9.85954, 
    viewOffsetY=2.756)
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON)
session.viewports['Viewport: 1'].view.setValues(nearPlane=637.147, 
    farPlane=946.682, width=161.521, height=90.3728, viewOffsetX=1.26771, 
    viewOffsetY=84.9556)
a = mdb.models['Model-1'].rootAssembly
v1 = a.instances['LEVEL1-13-1'].vertices
verts1 = v1.getSequenceFromMask(mask=('[#0 #8 ]', ), )
region = a.Set(vertices=verts1, name='Set-1')
mdb.models['Model-1'].DisplacementBC(name='BC-1', createStepName='Initial', 
    region=region, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET, 
    amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)
session.viewports['Viewport: 1'].view.setValues(nearPlane=706.029, 
    farPlane=836.782, width=178.983, height=100.143, cameraPosition=(-62.8115, 
    415.105, 733.943), cameraUpVector=(-0.487277, 0.673115, -0.556307), 
    cameraTarget=(-58.8944, 184.09, -23.5166), viewOffsetX=1.40477, 
    viewOffsetY=94.1403)
session.viewports['Viewport: 1'].view.setValues(nearPlane=697.977, 
    farPlane=844.835, width=276.22, height=154.548, viewOffsetX=34.074, 
    viewOffsetY=86.5125)
session.viewports['Viewport: 1'].view.setValues(nearPlane=590.666, 
    farPlane=825.267, width=233.752, height=130.787, cameraPosition=(-388.603, 
    -81.1048, 576.963), cameraUpVector=(-0.490064, 0.805101, -0.334141), 
    cameraTarget=(-22.9102, 202.694, -65.5754), viewOffsetX=28.8353, 
    viewOffsetY=73.2115)
session.viewports['Viewport: 1'].view.setValues(nearPlane=597.296, 
    farPlane=818.637, width=184.551, height=103.258, viewOffsetX=39.2966, 
    viewOffsetY=78.6079)
session.viewports['Viewport: 1'].view.setValues(nearPlane=559.909, 
    farPlane=809.092, width=172.999, height=96.7947, cameraPosition=(-235.35, 
    -196.102, 550.487), cameraUpVector=(-0.569063, 0.820527, -0.0538697), 
    cameraTarget=(-38.6833, 219.68, -94.1651), viewOffsetX=36.8369, 
    viewOffsetY=73.6875)
session.viewports['Viewport: 1'].view.setValues(nearPlane=588.438, 
    farPlane=815.63, width=181.814, height=101.727, cameraPosition=(-367.044, 
    -112.507, 566.415), cameraUpVector=(-0.462406, 0.857667, -0.224917), 
    cameraTarget=(-34.2659, 205.725, -77.8792), viewOffsetX=38.7139, 
    viewOffsetY=77.4422)
session.viewports['Viewport: 1'].view.setValues(nearPlane=588.629, 
    farPlane=815.439, width=160.703, height=89.9151, viewOffsetX=34.5926, 
    viewOffsetY=79.0319)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF, adaptiveMeshConstraints=ON)
mdb.models['Model-1'].ExplicitDynamicsStep(name='Step-1', previous='Initial')
session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Step-1')
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON, adaptiveMeshConstraints=OFF)
session.viewports['Viewport: 1'].view.setValues(nearPlane=594.017, 
    farPlane=810.05, width=99.2608, height=55.5375, viewOffsetX=23.2774, 
    viewOffsetY=81.3178)
a = mdb.models['Model-1'].rootAssembly
v1 = a.instances['LEVEL1-13-1'].vertices
verts1 = v1.getSequenceFromMask(mask=('[#200000 ]', ), )
region = a.Set(vertices=verts1, name='Set-2')
mdb.models['Model-1'].ConcentratedForce(name='Load-1', createStepName='Step-1', 
    region=region, cf1=-20.0, cf2=-20.0, distributionType=UNIFORM, field='', 
    localCsys=None)
session.viewports['Viewport: 1'].view.setValues(nearPlane=592.869, 
    farPlane=811.199, width=126.89, height=70.9961, viewOffsetX=33.6651, 
    viewOffsetY=80.8695)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF)
mdb.Job(name='F-20sqrt2', model='Model-1', description='', type=ANALYSIS, 
    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    memoryUnits=PERCENTAGE, explicitPrecision=SINGLE, 
    nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF, 
    contactPrint=OFF, historyPrint=OFF, userSubroutine='', scratch='', 
    resultsFormat=ODB, parallelizationMethodExplicit=DOMAIN, numDomains=1, 
    activateLoadBalancing=False, multiprocessingMode=DEFAULT, numCpus=1)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=ON)
mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=(
    'S', 'SVAVG', 'PE', 'PEVAVG', 'PEEQ', 'PEEQVAVG', 'LE', 'U', 'UT', 'UR', 
    'V', 'A', 'RF', 'CSTRESS', 'EVF'))
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=OFF)
mdb.jobs['F-20sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-20sqrt2.inp" has been submitted for analysis.
#: Error in job F-20sqrt2: 533 elements have missing property definitions. The elements have been identified in element set ErrElemMissingSection.
#: Job F-20sqrt2: Analysis Input File Processor aborted due to errors.
#: Error in job F-20sqrt2: Analysis Input File Processor exited with an error.
#: Job F-20sqrt2 aborted due to errors.
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON, mesh=OFF)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=OFF)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
mdb.models['Model-1'].HomogeneousSolidSection(name='Section-2', material='Al', 
    thickness=None)
del mdb.models['Model-1'].parts['LEVEL1-13'].sectionAssignments[0]
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
region = p.Set(cells=cells, name='Set-2')
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.SectionAssignment(region=region, sectionName='Section-2', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.jobs['F-20sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-20sqrt2.inp" has been submitted for analysis.
#: Job F-20sqrt2: Analysis Input File Processor completed successfully.
#: Job F-20sqrt2: Abaqus/Explicit Packager completed successfully.
#: Job F-20sqrt2: Abaqus/Explicit completed successfully.
#: Job F-20sqrt2 completed successfully. 
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/multiLevel_1/drag-200N/F-20sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/multiLevel_1/drag-200N/F-20sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       2
#: Number of Node Sets:          4
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_UNDEF, CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=280.575, 
    farPlane=342.229, width=222.64, height=124.569, cameraPosition=(-45.2259, 
    247.981, 305.947), cameraUpVector=(-0.0727011, 0.981777, -0.175583), 
    cameraTarget=(-78.2282, 296.959, -2.23112))
session.viewports['Viewport: 1'].view.setValues(nearPlane=277.167, 
    farPlane=344.632, width=219.936, height=123.056, cameraPosition=(-99.3169, 
    232.335, 304.064), cameraUpVector=(-0.0460972, 0.9897, -0.13553), 
    cameraTarget=(-77.8141, 297.079, -2.2167))
session.viewports['Viewport: 1'].view.setValues(nearPlane=274.029, 
    farPlane=347.771, width=246.091, height=137.691, viewOffsetX=7.32108, 
    viewOffsetY=0.609536)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    SYMBOLS_ON_DEF, ))
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    ORIENT_ON_DEF, ))
#: Warning: Material orientation information is not available in the current frame for any elements in the current display group.  Please make sure the primary variable is element-based AND orientations had been defined in the pertinent solid/shell sections.
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_UNDEF, CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=273.836, 
    farPlane=347.964, width=278.314, height=155.719, viewOffsetX=9.00137, 
    viewOffsetY=1.93191)
session.viewports['Viewport: 1'].view.setValues(nearPlane=277.368, 
    farPlane=347.748, width=281.903, height=157.728, cameraPosition=(-62.1114, 
    244.752, 308.095), cameraUpVector=(-0.0909031, 0.981435, -0.168884), 
    cameraTarget=(-78.1546, 296.66, -0.952212), viewOffsetX=9.11746, 
    viewOffsetY=1.95683)
session.viewports['Viewport: 1'].view.setValues(width=264.706, height=148.106, 
    viewOffsetX=8.16015, viewOffsetY=1.54991)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=264.001, 
    farPlane=355.803, width=252.219, height=141.119, cameraPosition=(-125.88, 
    209.127, 294.288), cameraUpVector=(-0.0368984, 0.997373, -0.0623304), 
    cameraTarget=(-77.7681, 297.198, -3.01805), viewOffsetX=7.77521, 
    viewOffsetY=1.4768)
session.viewports['Viewport: 1'].view.setValues(nearPlane=280.157, 
    farPlane=343.689, width=267.654, height=149.755, cameraPosition=(-76.4973, 
    237.964, 306.763), cameraUpVector=(-0.075544, 0.985551, -0.1516), 
    cameraTarget=(-78.5362, 296.519, -1.50484), viewOffsetX=8.25104, 
    viewOffsetY=1.56718)
session.viewports['Viewport: 1'].view.setValues(nearPlane=259.382, 
    farPlane=376.508, width=247.806, height=138.65, cameraPosition=(72.8022, 
    354.52, 272.765), cameraUpVector=(-0.273901, 0.863943, -0.422588), 
    cameraTarget=(-78.1649, 296.229, 3.92921), viewOffsetX=7.63919, 
    viewOffsetY=1.45097)
session.animationController.setValues(animationType=SCALE_FACTOR, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.animationController.setValues(animationType=NONE)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON)
session.viewports['Viewport: 1'].view.setValues(nearPlane=590.184, 
    farPlane=813.884, width=142.955, height=79.9848, viewOffsetX=36.0811, 
    viewOffsetY=83.9277)
import os
os.chdir(
    r"E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\F-2000")
mdb.models['Model-1'].loads['Load-1'].setValues(cf1=-200.0, cf2=-200.0, 
    distributionType=UNIFORM, field='')
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF)
mdb.jobs.changeKey(fromName='F-20sqrt2', toName='F-200sqrt2')
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Job F-200sqrt2: Abaqus/Explicit Packager completed successfully.
#: Job F-200sqrt2: Abaqus/Explicit completed successfully.
#: Job F-200sqrt2 completed successfully. 
session.viewports['Viewport: 1'].setValues(
    displayedObject=session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/multiLevel_1/drag-200N/F-20sqrt2.odb'])
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       2
#: Number of Node Sets:          4
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=235.742, 
    farPlane=391.833, width=175.84, height=98.3846, viewOffsetX=5.40884, 
    viewOffsetY=1.32185)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_UNDEF, CONTOURS_ON_DEF, ))
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=ON)
mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=(
    'S', 'SVAVG', 'E', 'VE', 'PE', 'PEVAVG', 'VEEQ', 'PEEQ', 'PEEQVAVG', 'LE', 
    'U', 'UT', 'UR', 'V', 'A', 'RF', 'CSTRESS', 'EVF'))
del mdb.models['Model-1'].steps['Step-1']
session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Initial')
mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial', 
    initialInc=0.001, nlgeom=ON)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Step-1')
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=OFF)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Job F-200sqrt2: Abaqus/Standard completed successfully.
#: Job F-200sqrt2 completed successfully. 
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       3
#: Number of Node Sets:          5
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_UNDEF, CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=280.225, 
    farPlane=343.616, width=222.363, height=124.414, cameraPosition=(7.88456, 
    297.932, 299.506), cameraUpVector=(-0.478061, 0.852119, -0.212956), 
    cameraTarget=(-78.2282, 296.959, -2.23114))
session.viewports['Viewport: 1'].view.setValues(nearPlane=271.811, 
    farPlane=352.033, width=215.686, height=120.678, cameraPosition=(7.79486, 
    334.228, 297.21), cameraUpVector=(-0.508954, 0.805355, -0.303922), 
    cameraTarget=(-78.2277, 296.742, -2.2174))
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON)
a = mdb.models['Model-1'].rootAssembly
v1 = a.instances['LEVEL1-13-1'].vertices
verts1 = v1.getSequenceFromMask(mask=('[#200000 ]', ), )
region = a.Set(vertices=verts1, name='Set-3')
mdb.models['Model-1'].ConcentratedForce(name='Load-1', createStepName='Step-1', 
    region=region, cf1=-200.0, cf2=-200.0, distributionType=UNIFORM, field='', 
    localCsys=None)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Error in job F-200sqrt2: Time increment required is less than the minimum specified
#: Error in job F-200sqrt2: Too many attempts made for this increment
#: Job F-200sqrt2: Abaqus/Standard aborted due to errors.
#: Error in job F-200sqrt2: Abaqus/Standard Analysis exited with an error - Please see the  message file for possible error messages if the file exists.
#: Job F-200sqrt2 aborted due to errors.
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF, mesh=ON)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=ON)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
pickedRegions = c.getSequenceFromMask(mask=('[#1 ]', ), )
p.deleteMesh(regions=pickedRegions)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
pickedRegions = c.getSequenceFromMask(mask=('[#1 ]', ), )
p.setMeshControls(regions=pickedRegions, elemShape=TET, technique=FREE)
elemType1 = mesh.ElemType(elemCode=C3D20R)
elemType2 = mesh.ElemType(elemCode=C3D15)
elemType3 = mesh.ElemType(elemCode=C3D10)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
pickedRegions =(cells, )
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    elemType3))
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.generateMesh()
session.viewports['Viewport: 1'].view.setValues(nearPlane=259.604, 
    farPlane=358.945, width=193.64, height=108.637, cameraPosition=(0.132996, 
    179.083, 274.688), cameraUpVector=(-0.133956, 0.9881, 0.0755973), 
    cameraTarget=(-82.9462, 295.058, -4.79327), viewOffsetX=10.6789, 
    viewOffsetY=2.98503)
mdb.save()
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\single.cae".
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Error in job F-200sqrt2: Too many attempts made for this increment
#: Job F-200sqrt2: Abaqus/Standard aborted due to errors.
#: Error in job F-200sqrt2: Abaqus/Standard Analysis exited with an error - Please see the  message file for possible error messages if the file exists.
#: Job F-200sqrt2 aborted due to errors.
session.viewports['Viewport: 1'].view.setValues(nearPlane=584.719, 
    farPlane=819.348, width=205.301, height=114.868, viewOffsetX=69.1049, 
    viewOffsetY=88.0934)
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON, mesh=OFF)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=OFF)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
mdb.models['Model-1'].HexagonalProfile(name='Profile-1', r=1.0, t=0.2)
mdb.models['Model-1'].BeamSection(name='Section-3', 
    integration=DURING_ANALYSIS, poissonRatio=0.0, profile='Profile-1', 
    material='Al', temperatureVar=LINEAR, consistentMassMatrix=False)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
del mdb.models['Model-1'].parts['LEVEL1-13'].sectionAssignments[0]
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF, datacheckJob=True)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Error in job F-200sqrt2: 3106 elements have missing property definitions. The elements have been identified in element set ErrElemMissingSection.
#: Job F-200sqrt2: Analysis Input File Processor aborted due to errors.
#: Error in job F-200sqrt2: Analysis Input File Processor exited with an error.
#: Job F-200sqrt2 aborted due to errors.
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
del mdb.models['Model-1'].sections['Section-1']
del mdb.models['Model-1'].sections['Section-2']
del mdb.models['Model-1'].sections['Section-3']
mdb.models['Model-1'].HomogeneousSolidSection(name='Section-1', material='Al', 
    thickness=None)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
region = p.Set(cells=cells, name='Set-5')
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF, mesh=ON)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=ON)
elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
pickedRegions =(cells, )
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    elemType3))
elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
pickedRegions =(cells, )
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    elemType3))
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.generateMesh()
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.deleteMesh()
elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
pickedRegions =(cells, )
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    elemType3))
elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
pickedRegions =(cells, )
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    elemType3))
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.generateMesh()
session.viewports['Viewport: 1'].view.setValues(nearPlane=257.51, 
    farPlane=361.04, width=218.27, height=122.455, viewOffsetX=10.2071, 
    viewOffsetY=3.4406)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
pickedRegions = c.getSequenceFromMask(mask=('[#1 ]', ), )
p.deleteMesh(regions=pickedRegions)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
pickedRegions = c.getSequenceFromMask(mask=('[#1 ]', ), )
p.setMeshControls(regions=pickedRegions, elemShape=HEX, technique=SWEEP, 
    algorithm=ADVANCING_FRONT)
elemType1 = mesh.ElemType(elemCode=C3D8R)
elemType2 = mesh.ElemType(elemCode=C3D6)
elemType3 = mesh.ElemType(elemCode=C3D4)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
pickedRegions =(cells, )
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    elemType3))
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.generateMesh()
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Error in job F-200sqrt2: Time increment required is less than the minimum specified
#: Error in job F-200sqrt2: Too many attempts made for this increment
#: Job F-200sqrt2: Abaqus/Standard aborted due to errors.
#: Error in job F-200sqrt2: Abaqus/Standard Analysis exited with an error - Please see the  message file for possible error messages if the file exists.
#: Job F-200sqrt2 aborted due to errors.
session.viewports['Viewport: 1'].view.setValues(nearPlane=583.611, 
    farPlane=820.457, width=217.992, height=121.969, viewOffsetX=69.7339, 
    viewOffsetY=88.5047)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=ON)
session.viewports['Viewport: 1'].view.setValues(nearPlane=582.348, 
    farPlane=821.719, width=261.888, height=146.529, viewOffsetX=72.9044, 
    viewOffsetY=90.503)
session.viewports['Viewport: 1'].partDisplay.setValues(mesh=OFF)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=OFF)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=ON)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
session.viewports['Viewport: 1'].view.setValues(nearPlane=218.727, 
    farPlane=387.071, width=185.9, height=104.013, cameraPosition=(-242.994, 
    103.476, -168.377), cameraUpVector=(-0.512918, 0.836512, 0.192774), 
    cameraTarget=(-60.5471, 303.959, -10.3213), viewOffsetX=8.66984, 
    viewOffsetY=2.92242)
session.viewports['Viewport: 1'].view.setValues(nearPlane=217.072, 
    farPlane=373.619, width=184.493, height=103.226, cameraPosition=(-368.248, 
    231.724, 28.6654), cameraUpVector=(0.137267, 0.989115, -0.0530034), 
    cameraTarget=(-64.1347, 292.721, -18.8366), viewOffsetX=8.60425, 
    viewOffsetY=2.90031)
session.viewports['Viewport: 1'].view.setValues(nearPlane=228.295, 
    farPlane=370.54, width=194.032, height=108.563, cameraPosition=(-293.134, 
    208.046, -191.889), cameraUpVector=(-0.0705042, 0.981498, 0.178018), 
    cameraTarget=(-59.2484, 297.081, -2.59485), viewOffsetX=9.0491, 
    viewOffsetY=3.05026)
session.viewports['Viewport: 1'].view.setValues(nearPlane=242.397, 
    farPlane=356.592, width=206.017, height=115.269, cameraPosition=(-57.7771, 
    93.4604, 219.351), cameraUpVector=(-0.295866, 0.883816, 0.362397), 
    cameraTarget=(-89.8228, 297.824, -16.5951), viewOffsetX=9.60805, 
    viewOffsetY=3.23867)
session.viewports['Viewport: 1'].view.setValues(nearPlane=247.204, 
    farPlane=351.785, width=128.072, height=71.6579, viewOffsetX=-10.3211, 
    viewOffsetY=-2.83497)
session.viewports['Viewport: 1'].partDisplay.setValues(mesh=ON)
p = mdb.models['Model-1'].parts['LEVEL1-13']
n = p.nodes
nodes = n.getSequenceFromMask(mask=('[#800 ]', ), )
p.Set(nodes=nodes, name='Set-4')
#: The set 'Set-4' has been created (1 node).
session.viewports['Viewport: 1'].partDisplay.setValues(mesh=OFF)
session.viewports['Viewport: 1'].view.setValues(nearPlane=242.423, 
    farPlane=356.566, width=182.801, height=102.279, viewOffsetX=-11.054, 
    viewOffsetY=-3.15934)
session.viewports['Viewport: 1'].view.setValues(nearPlane=239.92, 
    farPlane=379.705, width=180.914, height=101.223, cameraPosition=(-147.59, 
    41.0991, 162.271), cameraUpVector=(-0.375481, 0.803238, 0.46241), 
    cameraTarget=(-87.6117, 297.552, -8.30598), viewOffsetX=-10.9399, 
    viewOffsetY=-3.12672)
session.viewports['Viewport: 1'].view.setValues(nearPlane=236.643, 
    farPlane=417.755, width=178.443, height=99.8404, cameraPosition=(-287.063, 
    44.0781, -25.4891), cameraUpVector=(-0.475131, 0.74931, 0.461287), 
    cameraTarget=(-94.607, 290.716, -1.14558), viewOffsetX=-10.7905, 
    viewOffsetY=-3.08401)
session.viewports['Viewport: 1'].view.setValues(nearPlane=239.076, 
    farPlane=409.04, width=180.278, height=100.867, cameraPosition=(-346.462, 
    145.716, 109.13), cameraUpVector=(-0.190301, 0.981671, 0.0103486), 
    cameraTarget=(-95.6129, 302.004, 3.72054), viewOffsetX=-10.9015, 
    viewOffsetY=-3.11572)
session.viewports['Viewport: 1'].view.setValues(nearPlane=273.266, 
    farPlane=342.3, width=206.059, height=115.292, cameraPosition=(-48.624, 
    215.57, 295.638), cameraUpVector=(0.179, 0.980191, -0.0847669), 
    cameraTarget=(-74.4675, 300.983, -5.19284), viewOffsetX=-12.4605, 
    viewOffsetY=-3.5613)
session.viewports['Viewport: 1'].view.setValues(nearPlane=219.36, 
    farPlane=382.043, width=165.41, height=92.5487, cameraPosition=(211.683, 
    364.214, -20.4372), cameraUpVector=(-0.526087, 0.815147, -0.242423), 
    cameraTarget=(-94.7291, 296.998, -12.9569), viewOffsetX=-10.0025, 
    viewOffsetY=-2.85877)
session.viewports['Viewport: 1'].view.setValues(nearPlane=251.94, 
    farPlane=348.027, width=189.977, height=106.294, cameraPosition=(-119.84, 
    162.664, 266.264), cameraUpVector=(0.25869, 0.951734, 0.165174), 
    cameraTarget=(-65.2976, 305.312, -7.85163), viewOffsetX=-11.4881, 
    viewOffsetY=-3.28337)
session.viewports['Viewport: 1'].view.setValues(nearPlane=216.179, 
    farPlane=380.712, width=163.011, height=91.2063, cameraPosition=(-173.471, 
    32.1665, 103.336), cameraUpVector=(0.022249, 0.643833, 0.764843), 
    cameraTarget=(-64.5202, 309.091, 3.81105), viewOffsetX=-9.85745, 
    viewOffsetY=-2.81732)
session.viewports['Viewport: 1'].view.setValues(nearPlane=235.149, 
    farPlane=361.923, width=177.315, height=99.2097, cameraPosition=(-329.731, 
    323.199, -163.211), cameraUpVector=(-0.132174, 0.503329, 0.853926), 
    cameraTarget=(-77.0827, 287.497, 19.4272), viewOffsetX=-10.7224, 
    viewOffsetY=-3.06454)
session.viewports['Viewport: 1'].view.setValues(nearPlane=222.781, 
    farPlane=376.343, width=167.989, height=93.9916, cameraPosition=(-372.809, 
    238.056, 36.8487), cameraUpVector=(0.323777, 0.494567, 0.80658), 
    cameraTarget=(-64.5635, 290.568, 10.5926), viewOffsetX=-10.1584, 
    viewOffsetY=-2.90336)
session.viewports['Viewport: 1'].view.setValues(nearPlane=224.984, 
    farPlane=372.08, width=169.65, height=94.9211, cameraPosition=(-103.806, 
    2.17303, 44.0915), cameraUpVector=(-0.141442, 0.460089, 0.876534), 
    cameraTarget=(-66.8403, 311.558, 6.99031), viewOffsetX=-10.2589, 
    viewOffsetY=-2.93207)
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON, adaptiveMeshConstraints=OFF)
session.viewports['Viewport: 1'].view.setValues(nearPlane=576.855, 
    farPlane=827.213, width=332.268, height=185.907, viewOffsetX=81.1261, 
    viewOffsetY=88.4447)
p = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].view.setValues(nearPlane=576.986, 
    farPlane=827.082, width=293.659, height=164.305, viewOffsetX=76.0153, 
    viewOffsetY=84.9539)
session.viewports['Viewport: 1'].view.setValues(nearPlane=706.719, 
    farPlane=848.127, width=359.686, height=201.248, cameraPosition=(-125.778, 
    372.462, 751.388), cameraUpVector=(-0.271365, 0.785498, -0.556196), 
    cameraTarget=(-107.603, 172.399, -14.623), viewOffsetX=93.1069, 
    viewOffsetY=104.055)
p = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
session.viewports['Viewport: 1'].view.setValues(nearPlane=237.659, 
    farPlane=361.595, width=179.208, height=100.269, cameraPosition=(-129.126, 
    96.4705, 218.363), cameraUpVector=(0.0358923, 0.911052, 0.410727), 
    cameraTarget=(-64.8484, 308.794, -3.55879), viewOffsetX=-10.8369, 
    viewOffsetY=-3.09726)
session.viewports['Viewport: 1'].view.setValues(nearPlane=239.323, 
    farPlane=359.932, width=140.896, height=78.8329, viewOffsetX=-18.9865, 
    viewOffsetY=-11.1067)
p = mdb.models['Model-1'].parts['LEVEL1-13']
v = p.vertices
verts = v.getSequenceFromMask(mask=('[#200000 ]', ), )
p.Set(vertices=verts, name='Set-6')
#: The set 'Set-6' has been created (1 vertex).
mdb.models['Model-1'].parts['LEVEL1-13'].sets.changeKey(fromName='Set-6', 
    toName='drag point')
mdb.models['Model-1'].parts['LEVEL1-13'].sets.changeKey(fromName='Set-5', 
    toName='totally')
del mdb.models['Model-1'].parts['LEVEL1-13'].sets['Set-4']
del mdb.models['Model-1'].parts['LEVEL1-13'].sets['Set-2']
del mdb.models['Model-1'].parts['LEVEL1-13'].sets['Set-1']
mdb.models['Model-1'].parts['LEVEL1-13'].sets.changeKey(fromName='totally', 
    toName='whole part')
session.viewports['Viewport: 1'].view.setValues(nearPlane=238.527, 
    farPlane=360.728, width=150.002, height=83.928, viewOffsetX=-9.50383, 
    viewOffsetY=-3.65945)
session.viewports['Viewport: 1'].view.setValues(nearPlane=242.524, 
    farPlane=358.634, width=152.516, height=85.3343, cameraPosition=(-24.4156, 
    506.908, 208.559), cameraUpVector=(0.0528401, 0.424331, -0.903964), 
    cameraTarget=(-69.0455, 288.503, -12.2796), viewOffsetX=-9.66308, 
    viewOffsetY=-3.72077)
session.viewports['Viewport: 1'].view.setValues(nearPlane=246.631, 
    farPlane=354.527, width=94.9303, height=53.1145, viewOffsetX=23.9624, 
    viewOffsetY=18.9982)
session.viewports['Viewport: 1'].view.setValues(nearPlane=247.948, 
    farPlane=404.321, width=95.4371, height=53.3981, cameraPosition=(98.328, 
    544.842, 113.749), cameraUpVector=(-0.106583, 0.104554, -0.988791), 
    cameraTarget=(-69.3651, 304.934, 0.675751), viewOffsetX=24.0903, 
    viewOffsetY=19.0996)
session.viewports['Viewport: 1'].view.setValues(nearPlane=248.827, 
    farPlane=403.442, width=79.5496, height=44.5088, viewOffsetX=23.8312, 
    viewOffsetY=18.6918)
p = mdb.models['Model-1'].parts['LEVEL1-13']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#4000000 ]', ), )
p.Set(faces=faces, name='Set-3')
#: The set 'Set-3' has been created (1 face).
mdb.models['Model-1'].parts['LEVEL1-13'].sets.changeKey(fromName='Set-3', 
    toName='fix phase')
session.viewports['Viewport: 1'].view.setValues(nearPlane=242.022, 
    farPlane=410.247, width=157.344, height=88.0357, viewOffsetX=19.7889, 
    viewOffsetY=19.3862)
session.viewports['Viewport: 1'].view.setValues(nearPlane=268.046, 
    farPlane=339.021, width=174.263, height=97.5021, cameraPosition=(18.1397, 
    304.098, 288.813), cameraUpVector=(-0.0906262, 0.897642, -0.431308), 
    cameraTarget=(-73.6124, 268.21, -9.10681), viewOffsetX=21.9167, 
    viewOffsetY=21.4707)
session.viewports['Viewport: 1'].view.setValues(nearPlane=217.976, 
    farPlane=328.105, width=141.711, height=79.2889, cameraPosition=(-259.361, 
    287.067, 209.775), cameraUpVector=(0.0104663, 0.844789, -0.534998), 
    cameraTarget=(-52.3579, 259.961, -24.4848), viewOffsetX=17.8227, 
    viewOffsetY=17.46)
session.viewports['Viewport: 1'].view.setValues(nearPlane=209.798, 
    farPlane=336.283, width=288.94, height=161.665, viewOffsetX=39.8067, 
    viewOffsetY=14.0341)
session.viewports['Viewport: 1'].view.setValues(nearPlane=274.857, 
    farPlane=335.192, width=378.542, height=211.799, cameraPosition=(-72.5024, 
    244.662, 302.871), cameraUpVector=(-0.410947, 0.873082, -0.262395), 
    cameraTarget=(-87.3445, 263.447, -10.0019), viewOffsetX=52.151, 
    viewOffsetY=18.3861)
session.viewports['Viewport: 1'].view.setValues(nearPlane=271.57, 
    farPlane=338.48, width=310.651, height=173.813, viewOffsetX=41.7336, 
    viewOffsetY=22.2859)
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=OFF)
mdb.models['Model-1'].sections.changeKey(fromName='Section-1', toName='Solid')
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF, adaptiveMeshConstraints=ON)
mdb.models['Model-1'].steps['Step-1'].setValues(maxNumInc=1000, 
    initialInc=0.01)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=OFF)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Error in job F-200sqrt2: 533 elements have missing property definitions. The elements have been identified in element set ErrElemMissingSection.
#: Job F-200sqrt2: Analysis Input File Processor aborted due to errors.
#: Error in job F-200sqrt2: Analysis Input File Processor exited with an error.
#: Job F-200sqrt2 aborted due to errors.
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=ON)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
import sys
sys.path.insert(46, 
    r'e:/2-profession/Abaqus/SIMULIA/CAE/2016/win_b64/code/python2.7/lib/abaqus_plugins/examples')
import caeExamples
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=OFF)
mdb.models['Model-1'].SurfaceSection(name='Ssurface', useDensity=ON, 
    density=2700.0)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
del mdb.models['Model-1'].parts['LEVEL1-13'].sectionAssignments[0]
p = mdb.models['Model-1'].parts['LEVEL1-13']
region = p.sets['whole part']
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.SectionAssignment(region=region, sectionName='Solid', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)
mdb.save()
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\single.cae".
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=ON)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=200.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
session.viewports['Viewport: 1'].view.setValues(nearPlane=135.612, 
    farPlane=241.512, width=602.187, height=336.93, cameraPosition=(34.918, 
    12.762, 188.562), cameraTarget=(34.918, 12.762, 0))
s.undo()
#* Nothing to undo.
s.undo()
#* Nothing to undo.
s.undo()
#* Nothing to undo.
s.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
session.viewports['Viewport: 1'].view.setValues(nearPlane=279.833, 
    farPlane=340.389, width=274.491, height=153.58, viewOffsetX=5.9152, 
    viewOffsetY=1.11056)
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON)
a = mdb.models['Model-1'].rootAssembly
region = a.instances['LEVEL1-13-1'].sets['drag point']
mdb.models['Model-1'].loads['Load-1'].setValues(region=region, 
    distributionType=UNIFORM, field='')
del mdb.models['Model-1'].boundaryConditions['BC-1']
a = mdb.models['Model-1'].rootAssembly
region = a.instances['LEVEL1-13-1'].sets['fix phase']
mdb.models['Model-1'].DisplacementBC(name='BC-1', createStepName='Step-1', 
    region=region, u1=0.0, u2=0.0, u3=0.0, ur1=0.0, ur2=0.0, ur3=UNSET, 
    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
    localCsys=None)
mdb.save()
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\single.cae".
mdb.save()
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\single.cae".
mdb.save()
#: The model database has been saved to "E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\single.cae".
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON)
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Job F-200sqrt2: Abaqus/Standard completed successfully.
#: Job F-200sqrt2 completed successfully. 
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF)
session.viewports['Viewport: 1'].setValues(
    displayedObject=session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb'])
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       3
#: Number of Node Sets:          7
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_UNDEF, CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=268.882, 
    farPlane=346.171, width=213.362, height=119.378, cameraPosition=(-3.68224, 
    287.344, 301.867), cameraUpVector=(-0.505389, 0.84145, -0.191164), 
    cameraTarget=(-77.6569, 297.982, -2.88986))
session.viewports['Viewport: 1'].view.setValues(nearPlane=249.36, 
    farPlane=363.966, width=197.871, height=110.71, cameraPosition=(-15.6248, 
    179.354, 280.468), cameraUpVector=(-0.450254, 0.886442, 0.107204), 
    cameraTarget=(-77.4138, 300.18, -2.45425))
session.viewports['Viewport: 1'].view.setValues(nearPlane=237.104, 
    farPlane=374.501, width=188.146, height=105.269, cameraPosition=(-35.9488, 
    79.1583, 215.609), cameraUpVector=(-0.394678, 0.812273, 0.429467), 
    cameraTarget=(-76.9417, 302.507, -0.947678))
session.viewports['Viewport: 1'].view.setValues(nearPlane=249.382, 
    farPlane=363.208, width=197.889, height=110.72, cameraPosition=(-28.8611, 
    155.033, 271.392), cameraUpVector=(-0.43083, 0.887155, 0.165355), 
    cameraTarget=(-77.1267, 300.526, -2.40405))
session.viewports['Viewport: 1'].view.setValues(nearPlane=243.242, 
    farPlane=369.349, width=248.231, height=138.888, viewOffsetX=6.38319, 
    viewOffsetY=3.04822)
session.viewports['Viewport: 1'].view.setValues(nearPlane=229.128, 
    farPlane=405.725, width=233.827, height=130.829, cameraPosition=(49.2301, 
    524.52, 181.881), cameraUpVector=(-0.67133, 0.386367, -0.632484), 
    cameraTarget=(-76.8494, 294.423, 9.77161), viewOffsetX=6.0128, 
    viewOffsetY=2.87135)
session.viewports['Viewport: 1'].view.setValues(nearPlane=238.102, 
    farPlane=398.485, width=242.986, height=135.953, cameraPosition=(-47.7963, 
    610.156, -32.5447), cameraUpVector=(-0.81394, -0.328543, -0.479126), 
    cameraTarget=(-75.539, 300.335, 8.72859), viewOffsetX=6.24831, 
    viewOffsetY=2.98381)
#: The contents of viewport "Viewport: 1" have been copied to the clipboard.
session.viewports['Viewport: 1'].view.setValues(nearPlane=255.239, 
    farPlane=375.006, width=260.475, height=145.738, cameraPosition=(-40.8526, 
    437.014, 283.018), cameraUpVector=(-0.503501, 0.593939, -0.627473), 
    cameraTarget=(-79.3776, 290.277, 8.34378), viewOffsetX=6.69802, 
    viewOffsetY=3.19857)
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.animationController.setValues(animationType=NONE)
session.animationController.animationOptions.setValues(frameRate=33)
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.viewports['Viewport: 1'].view.setValues(nearPlane=221.523, 
    farPlane=416.143, width=226.067, height=126.487, cameraPosition=(104.169, 
    529.196, 118.028), cameraUpVector=(-0.355315, 0.25035, -0.900598), 
    cameraTarget=(-79.2321, 298.576, 10.1378), viewOffsetX=5.81323, 
    viewOffsetY=2.77605)
session.viewports['Viewport: 1'].view.setValues(nearPlane=241.696, 
    farPlane=391.689, width=246.654, height=138.005, cameraPosition=(-81.7205, 
    611.279, -24.0378), cameraUpVector=(-0.0981308, -0.4202, -0.90211), 
    cameraTarget=(-84.104, 298.921, 5.77205), viewOffsetX=6.3426, 
    viewOffsetY=3.02885)
session.viewports['Viewport: 1'].view.setValues(nearPlane=232.057, 
    farPlane=398.192, width=236.817, height=132.501, cameraPosition=(-132.945, 
    589.129, 105.954), cameraUpVector=(-0.169205, -0.0541269, -0.984093), 
    cameraTarget=(-83.7567, 295.32, 7.37059), viewOffsetX=6.08965, 
    viewOffsetY=2.90806)
session.viewports['Viewport: 1'].view.setValues(nearPlane=239.721, 
    farPlane=390.823, width=244.638, height=136.877, cameraPosition=(-101.452, 
    578.039, 142.392), cameraUpVector=(-0.254965, 0.0727294, -0.964211), 
    cameraTarget=(-83.3681, 295.163, 7.79585), viewOffsetX=6.29077, 
    viewOffsetY=3.00411)
session.viewports['Viewport: 1'].view.setValues(nearPlane=233.842, 
    farPlane=397.123, width=238.638, height=133.52, cameraPosition=(-120.166, 
    599.979, 77.1065), cameraUpVector=(-0.0903304, -0.126349, -0.987865), 
    cameraTarget=(-84.3031, 296.282, 6.79052), viewOffsetX=6.13649, 
    viewOffsetY=2.93044)
session.viewports['Viewport: 1'].view.setValues(nearPlane=240.773, 
    farPlane=392.345, width=245.711, height=137.477, cameraPosition=(91.7181, 
    406.881, 244.918), cameraUpVector=(-0.782021, 0.597409, -0.177607), 
    cameraTarget=(-77.6025, 289.897, 8.04715), viewOffsetX=6.31837, 
    viewOffsetY=3.0173)
session.viewports['Viewport: 1'].view.setValues(nearPlane=259.976, 
    farPlane=371.536, width=265.307, height=148.441, cameraPosition=(67.3116, 
    268.568, 281.357), cameraUpVector=(-0.736352, 0.67427, 0.0560794), 
    cameraTarget=(-78.6168, 287.378, 4.20441), viewOffsetX=6.82228, 
    viewOffsetY=3.25794)
session.viewports['Viewport: 1'].view.setValues(nearPlane=242.607, 
    farPlane=390.688, width=247.582, height=138.524, cameraPosition=(113.732, 
    360.966, 245.13), cameraUpVector=(-0.795928, 0.605232, 0.0139149), 
    cameraTarget=(-77.0336, 288.391, 6.7946), viewOffsetX=6.36648, 
    viewOffsetY=3.04028)
session.viewports['Viewport: 1'].view.setValues(nearPlane=264.63, 
    farPlane=361.755, width=270.056, height=151.099, cameraPosition=(-181.29, 
    265.908, 298.571), cameraUpVector=(0.0458529, 0.966272, -0.253408), 
    cameraTarget=(-85.7829, 291.522, 0.771454), viewOffsetX=6.9444, 
    viewOffsetY=3.31626)
session.viewports['Viewport: 1'].view.setValues(nearPlane=268.562, 
    farPlane=358.06, width=274.069, height=153.344, cameraPosition=(-163.542, 
    302.202, 305.588), cameraUpVector=(0.0397983, 0.929191, -0.367452), 
    cameraTarget=(-86.0127, 291.283, 1.72567), viewOffsetX=7.04759, 
    viewOffsetY=3.36554)
session.viewports['Viewport: 1'].view.setValues(nearPlane=261.679, 
    farPlane=365.551, width=267.045, height=149.414, cameraPosition=(-119.647, 
    196.179, 298.365), cameraUpVector=(-0.0779011, 0.996092, -0.0416157), 
    cameraTarget=(-85.9385, 291.408, 1.28386), viewOffsetX=6.86696, 
    viewOffsetY=3.27928)
session.viewports['Viewport: 1'].view.setValues(nearPlane=273.292, 
    farPlane=354.021, width=278.896, height=156.045, cameraPosition=(-115.719, 
    256.108, 312.703), cameraUpVector=(-0.00643157, 0.974091, -0.226067), 
    cameraTarget=(-86.3231, 291.645, 2.32411), viewOffsetX=7.1717, 
    viewOffsetY=3.42481)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=ON)
mdb.models['Model-1'].steps['Step-1'].setValues(timePeriod=5.0, 
    initialInc=0.001, minInc=5e-05, maxInc=5.0)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=OFF)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Job F-200sqrt2: Abaqus/Standard completed successfully.
#: Job F-200sqrt2 completed successfully. 
session.viewports['Viewport: 1'].setValues(
    displayedObject=session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb'])
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       3
#: Number of Node Sets:          7
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_UNDEF, CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=240.6, 
    farPlane=376.748, width=190.919, height=106.822, cameraPosition=(84.4097, 
    203.459, 248.441), cameraUpVector=(-0.733251, 0.614708, 0.29065), 
    cameraTarget=(-78.129, 298.431, -2.60361))
session.viewports['Viewport: 1'].view.setValues(nearPlane=254.922, 
    farPlane=360.272, width=202.284, height=113.181, cameraPosition=(27.0399, 
    236.873, 286.386), cameraUpVector=(-0.792534, 0.607604, 0.0520268), 
    cameraTarget=(-77.1788, 297.878, -3.23207))
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.viewports['Viewport: 1'].view.setValues(nearPlane=218.406, 
    farPlane=404.818, width=173.31, height=96.9686, cameraPosition=(158.991, 
    468.864, 103.263), cameraUpVector=(-0.830757, 0.555436, -0.0365179), 
    cameraTarget=(-79.834, 293.21, 0.452789))
session.viewports['Viewport: 1'].view.setValues(nearPlane=257.582, 
    farPlane=363.019, width=204.397, height=114.362, cameraPosition=(93.165, 
    298.395, 261.41), cameraUpVector=(-0.772661, 0.626888, 0.100033), 
    cameraTarget=(-79.3747, 294.399, -0.650543))
session.viewports['Viewport: 1'].view.setValues(nearPlane=252.226, 
    farPlane=367.323, width=200.147, height=111.984, cameraPosition=(8.45714, 
    194.851, 283.306), cameraUpVector=(-0.735984, 0.670369, 0.0945134), 
    cameraTarget=(-78.4232, 295.562, -0.896477))
session.viewports['Viewport: 1'].view.setValues(nearPlane=271.983, 
    farPlane=346.004, width=215.825, height=120.756, cameraPosition=(-25.6578, 
    276.52, 307.658), cameraUpVector=(-0.738052, 0.651473, -0.175677), 
    cameraTarget=(-77.9814, 294.504, -1.21186))
session.viewports['Viewport: 1'].view.setValues(nearPlane=273.069, 
    farPlane=344.703, width=216.687, height=121.238, cameraPosition=(-33.203, 
    282.938, 309.145), cameraUpVector=(-0.728976, 0.652173, -0.208003), 
    cameraTarget=(-77.8644, 294.404, -1.23492))
import os
os.chdir(r"E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation")
session.viewports['Viewport: 1'].view.setValues(nearPlane=274.072, 
    farPlane=343.7, width=192.168, height=107.52, viewOffsetX=3.73167, 
    viewOffsetY=4.74661)
import os
os.chdir(
    r"E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\drag-200sqrt2")
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Job F-200sqrt2: Abaqus/Standard completed successfully.
#: Job F-200sqrt2 completed successfully. 
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=ON)
regionDef=mdb.models['Model-1'].rootAssembly.allInstances['LEVEL1-13-1'].sets['drag point']
mdb.models['Model-1'].historyOutputRequests['H-Output-1'].setValues(variables=(
    'U1', 'U2', 'U3', 'UR1', 'UR2', 'UR3', 'UT', 'UR', 'V1', 'V2', 'V3', 'VR1', 
    'VR2', 'VR3', 'VT', 'VR', 'WARP', 'RBANG', 'RBROT', 'IRA1', 'IRA2', 'IRA3', 
    'IRAR1', 'IRAR2', 'IRAR3', 'ALLAE', 'ALLCD', 'ALLDMD', 'ALLEE', 'ALLFD', 
    'ALLIE', 'ALLJD', 'ALLKE', 'ALLKL', 'ALLPD', 'ALLQB', 'ALLSE', 'ALLSD', 
    'ALLVD', 'ALLWK', 'ETOTAL'), region=regionDef, sectionPoints=DEFAULT, 
    rebar=EXCLUDE)
mdb.models['Model-1'].HistoryOutputRequest(name='H-Output-2', 
    createStepName='Step-1', variables=('IRA1', 'IRA2', 'IRA3', 'IRAR1', 
    'IRAR2', 'IRAR3', 'IRF1', 'IRF2', 'IRF3', 'IRM1', 'IRM2', 'IRM3', 'ALLAE', 
    'ALLCD', 'ALLDMD', 'ALLEE', 'ALLFD', 'ALLIE', 'ALLJD', 'ALLKE', 'ALLKL', 
    'ALLPD', 'ALLQB', 'ALLSE', 'ALLSD', 'ALLVD', 'ALLWK', 'ETOTAL'), 
    region=MODEL, sectionPoints=DEFAULT, rebar=EXCLUDE)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=OFF)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Job F-200sqrt2: Abaqus/Standard completed successfully.
#: Job F-200sqrt2 completed successfully. 
session.viewports['Viewport: 1'].setValues(
    displayedObject=session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb'])
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       3
#: Number of Node Sets:          7
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
odb = session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb']
xy1 = xyPlot.XYDataFromHistory(odb=odb, 
    outputVariableName='Spatial displacement: U1 at Node 12 in NSET drag point', 
    )
c1 = session.Curve(xyData=xy1)
xyp = session.XYPlot('XYPlot-1')
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
chart.setValues(curvesToPlot=(c1, ), )
session.viewports['Viewport: 1'].setValues(displayedObject=xyp)
odb = session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb']
xy1 = xyPlot.XYDataFromHistory(odb=odb, 
    outputVariableName='Spatial displacement: U2 at Node 12 in NSET drag point', 
    )
c1 = session.Curve(xyData=xy1)
xyp = session.xyPlots['XYPlot-1']
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
chart.setValues(curvesToPlot=(c1, ), )
odb = session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb']
session.viewports['Viewport: 1'].setValues(displayedObject=odb)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_UNDEF, CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=228.888, 
    farPlane=395.34, width=149.533, height=89.7929, viewOffsetX=-2.96246, 
    viewOffsetY=3.19252)
session.viewports['Viewport: 1'].view.setValues(nearPlane=273.319, 
    farPlane=341.801, width=178.559, height=107.223, cameraPosition=(-3.52448, 
    300.454, 301.985), cameraUpVector=(-0.63769, 0.746503, -0.189958), 
    cameraTarget=(-76.5892, 300.501, -3.17686), viewOffsetX=-3.53752, 
    viewOffsetY=3.81224)
session.viewports['Viewport: 1'].view.setValues(nearPlane=258.144, 
    farPlane=352.948, width=168.645, height=101.27, cameraPosition=(-50.4278, 
    192.647, 290.346), cameraUpVector=(-0.691391, 0.722064, -0.0245508), 
    cameraTarget=(-75.419, 303.631, -2.0923), viewOffsetX=-3.34111, 
    viewOffsetY=3.60057)
session.viewports['Viewport: 1'].view.setValues(nearPlane=282.203, 
    farPlane=332.47, width=184.363, height=110.709, cameraPosition=(-75.3677, 
    288.366, 311.403), cameraUpVector=(-0.567065, 0.764683, -0.306098), 
    cameraTarget=(-74.8162, 300.038, -2.16644), viewOffsetX=-3.65251, 
    viewOffsetY=3.93615)
session.viewports['Viewport: 1'].view.setValues(nearPlane=280.809, 
    farPlane=333.865, width=183.453, height=110.162, cameraPosition=(-77.216, 
    285.046, 311.276), cameraUpVector=(0.0156971, 0.954434, -0.29801), 
    cameraTarget=(-76.6646, 296.718, -2.29327), viewOffsetX=-3.63447, 
    viewOffsetY=3.91671)
session.viewports['Viewport: 1'].view.setValues(nearPlane=285.07, 
    farPlane=329.603, width=128.48, height=77.1507, viewOffsetX=-21.6938, 
    viewOffsetY=-32.9683)
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.animationController.setValues(animationType=NONE)
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.animationController.animationOptions.setValues(frameRate=58)
session.animationController.setValues(animationType=NONE)
odb = session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb']
xy1 = xyPlot.XYDataFromHistory(odb=odb, 
    outputVariableName='Spatial displacement: U1 at Node 12 in NSET drag point', 
    )
c1 = session.Curve(xyData=xy1)
xyp = session.xyPlots['XYPlot-1']
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
chart.setValues(curvesToPlot=(c1, ), )
session.viewports['Viewport: 1'].setValues(displayedObject=xyp)
odb = session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb']
xy1 = xyPlot.XYDataFromHistory(odb=odb, 
    outputVariableName='Spatial displacement: U2 at Node 12 in NSET drag point', 
    )
c1 = session.Curve(xyData=xy1)
xyp = session.xyPlots['XYPlot-1']
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
chart.setValues(curvesToPlot=(c1, ), )
odb = session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb']
xy1 = xyPlot.XYDataFromHistory(odb=odb, 
    outputVariableName='Spatial displacement: U3 at Node 12 in NSET drag point', 
    )
c1 = session.Curve(xyData=xy1)
xyp = session.xyPlots['XYPlot-1']
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
chart.setValues(curvesToPlot=(c1, ), )
o7 = session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb']
session.viewports['Viewport: 1'].setValues(displayedObject=o7)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=10)
session.viewports['Viewport: 1'].view.setValues(nearPlane=292.352, 
    farPlane=329.926, width=191.776, height=115.16, viewOffsetX=-7.82098, 
    viewOffsetY=-28.3547)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=23)
#: The contents of viewport "Viewport: 1" have been copied to the clipboard.
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=26)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=14)
session.viewports['Viewport: 1'].view.setValues(width=203.336, height=122.101, 
    viewOffsetX=-5.69368, viewOffsetY=-28.2998)
session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/F-2000/F-200sqrt2.odb'].close(
    )
session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/multiLevel_1/drag-200N/F-20sqrt2.odb'].close(
    )
session.viewports['Viewport: 1'].setValues(
    displayedObject=session.mdbData['Model-1'])
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Initial')
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1')
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1')
session.viewports['Viewport: 1'].odbDisplay.contourOptions.setValues(
    spectrum='Black to white')
session.viewports['Viewport: 1'].odbDisplay.contourOptions.setValues(
    spectrum='Blue to red')
session.viewports['Viewport: 1'].odbDisplay.contourOptions.setValues(
    spectrum='Rainbow')
x = session.xyPlots['XYPlot-1']
session.viewports['Viewport: 1'].setValues(displayedObject=x)
session.curves['_temp_1'].setValues(useDefault=True, legendSource=CURVE_NAME)
session.curves['_temp_1'].setValues(useDefault=True, legendSource=CURVE_LEGEND)
session.curves['_temp_1'].setValues(useDefault=True, legendSource=CURVE_NAME)
session.curves['_temp_1'].lineStyle.setValues(style=DASHED)
session.curves['_temp_1'].symbolStyle.setValues(size=1.6)
session.curves['_temp_1'].symbolStyle.setValues(size=2)
session.charts['Chart-1'].axes1[0].axisData.setValues(scale=DB)
session.charts['Chart-1'].axes1[0].axisData.setValues(scale=LINEAR)
session.charts['Chart-1'].axes1[0].axisData.setValues(scale=DB)
session.charts['Chart-1'].axes1[0].axisData.setValues(scale=LINEAR)
session.charts['Chart-1'].axes1[0].axisData.setValues(scale=DB)
session.charts['Chart-1'].axes1[0].axisData.setValues(scale=LINEAR)
session.charts['Chart-1'].axes1[0].axisData.setValues(scale=LOG)
session.charts['Chart-1'].axes1[0].axisData.setValues(scale=LINEAR)
session.charts['Chart-1'].axes1[0].axisData.setValues(tickMode=INCREMENT)
session.charts['Chart-1'].axes1[0].axisData.setValues(tickMode=AUTOCOMPUTE)
xyp = session.xyPlots['XYPlot-1']
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
xy1 = session.xyDataObjects['_temp_1']
c1 = session.Curve(xyData=xy1)
chart.setValues(curvesToPlot=(c1, ), )
odb = session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb']
xy1 = xyPlot.XYDataFromHistory(odb=odb, 
    outputVariableName='Static dissipation (stabilization): ALLSD for Whole Model', 
    )
c1 = session.Curve(xyData=xy1)
xyp = session.xyPlots['XYPlot-1']
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
chart.setValues(curvesToPlot=(c1, ), )
odb = session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb']
xy1 = xyPlot.XYDataFromHistory(odb=odb, 
    outputVariableName='Spatial displacement: U2 at Node 12 in NSET drag point', 
    )
c1 = session.Curve(xyData=xy1)
xyp = session.xyPlots['XYPlot-1']
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
chart.setValues(curvesToPlot=(c1, ), )
#: The contents of viewport "Viewport: 1" have been copied to the clipboard.
xyp = session.xyPlots['XYPlot-1']
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
xy1 = session.xyDataObjects['_temp_1']
c1 = session.Curve(xyData=xy1)
chart.setValues(curvesToPlot=(c1, ), )
xyp = session.xyPlots['XYPlot-1']
chartName = xyp.charts.keys()[0]
chart = xyp.charts[chartName]
xy1 = session.xyDataObjects['_temp_1']
c1 = session.Curve(xyData=xy1)
chart.setValues(curvesToPlot=(c1, ), )
session.viewports['Viewport: 1'].partDisplay.setValues(mesh=ON)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=ON)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=OFF)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
#: 
#: Part instance:  LEVEL1-13,  Number of regions verified:  1
#:   Number of elements :  533,   Analysis errors:  0 (0%),  Analysis warnings:  0 (0%)
session.viewports['Viewport: 1'].view.setValues(nearPlane=289.989, 
    farPlane=330.233, width=164.999, height=99.3685, viewOffsetX=7.7815, 
    viewOffsetY=16.4735)
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.deleteMesh()
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.seedPart(size=2.0, deviationFactor=0.1, minSizeFactor=0.1)
session.viewports['Viewport: 1'].view.setValues(nearPlane=290.067, 
    farPlane=330.155, width=145.832, height=87.8256, viewOffsetX=5.14993, 
    viewOffsetY=17.42)
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.seedPart(size=1.5, deviationFactor=0.1, minSizeFactor=0.1)
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.generateMesh()
session.viewports['Viewport: 1'].view.setValues(nearPlane=286.164, 
    farPlane=334.059, width=187.305, height=112.802, viewOffsetX=13.5132, 
    viewOffsetY=-4.91013)
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.jobs['F-200sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-200sqrt2.inp" has been submitted for analysis.
#: Job F-200sqrt2: Analysis Input File Processor completed successfully.
#: Job F-200sqrt2: Abaqus/Standard completed successfully.
#: Job F-200sqrt2 completed successfully. 
session.viewports['Viewport: 1'].view.setValues(nearPlane=690.965, 
    farPlane=863.881, width=479.174, height=287.74, viewOffsetX=97.4919, 
    viewOffsetY=110.962)
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       3
#: Number of Node Sets:          7
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_UNDEF, CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=254.248, 
    farPlane=366.922, width=228.181, height=137.02, viewOffsetX=4.69739, 
    viewOffsetY=-6.29738)
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.animationController.animationOptions.setValues(frameRate=81)
session.animationController.setValues(animationType=NONE)
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.animationController.setValues(animationType=NONE)
session.animationController.animationOptions.setValues(frameRate=100)
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_UNDEF, CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=283.027, 
    farPlane=338.97, width=270.223, height=162.266, viewOffsetX=24.1913, 
    viewOffsetY=-6.09665)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
session.viewports['Viewport: 1'].view.setValues(nearPlane=205.232, 
    farPlane=365.29, width=195.947, height=117.664, cameraPosition=(-305.242, 
    372.073, 160.289), cameraUpVector=(0.322669, 0.799476, -0.506678), 
    cameraTarget=(-64.5311, 289.426, -23.2588), viewOffsetX=17.5419, 
    viewOffsetY=-4.42088)
session.viewports['Viewport: 1'].view.setValues(nearPlane=263.948, 
    farPlane=334.564, width=252.006, height=151.327, cameraPosition=(-52.3817, 
    354.836, 293.204), cameraUpVector=(0.033409, 0.864061, -0.502278), 
    cameraTarget=(-99.0197, 300.136, -12.2384), viewOffsetX=22.5605, 
    viewOffsetY=-5.68567)
session.viewports['Viewport: 1'].view.setValues(nearPlane=247.232, 
    farPlane=356.81, width=236.046, height=141.743, cameraPosition=(-20.0638, 
    200.472, 280.655), cameraUpVector=(0.135654, 0.990275, -0.0308695), 
    cameraTarget=(-98.4106, 307.967, -3.54321), viewOffsetX=21.1317, 
    viewOffsetY=-5.32558)
session.viewports['Viewport: 1'].view.setValues(nearPlane=260.973, 
    farPlane=332.447, width=249.165, height=149.621, cameraPosition=(-172.884, 
    307.127, 283.08), cameraUpVector=(0.159901, 0.934996, -0.316567), 
    cameraTarget=(-94.1377, 302.697, -20.6322), viewOffsetX=22.3062, 
    viewOffsetY=-5.62158)
session.animationController.setValues(animationType=NONE)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=40)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=44)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=52)
session.viewports['Viewport: 1'].view.setValues(nearPlane=209.798, 
    farPlane=384.075, width=200.306, height=120.282, cameraPosition=(-37.7012, 
    163.671, 267.422), cameraUpVector=(0.0229946, 0.990675, 0.13429), 
    cameraTarget=(-101.749, 307.528, -3.99042), viewOffsetX=17.9321, 
    viewOffsetY=-4.51924)
session.viewports['Viewport: 1'].view.setValues(nearPlane=233.327, 
    farPlane=361.214, width=222.771, height=133.772, cameraPosition=(-131.466, 
    264.495, 293.001), cameraUpVector=(0.0980369, 0.974839, -0.200192), 
    cameraTarget=(-97.1137, 304.821, -16.2812), viewOffsetX=19.9432, 
    viewOffsetY=-5.02608)
session.viewports['Viewport: 1'].view.setValues(nearPlane=222.068, 
    farPlane=371.784, width=212.021, height=127.317, cameraPosition=(-82.6204, 
    195.726, 283.188), cameraUpVector=(-0.0059618, 0.999791, 0.0195552), 
    cameraTarget=(-100.35, 305.973, -10.0578), viewOffsetX=18.9808, 
    viewOffsetY=-4.78354)
session.viewports['Viewport: 1'].view.setValues(nearPlane=235.286, 
    farPlane=359.625, width=224.641, height=134.895, cameraPosition=(-145.317, 
    292.516, 291.44), cameraUpVector=(0.149606, 0.948663, -0.278669), 
    cameraTarget=(-95.7517, 304, -18.1941), viewOffsetX=20.1106, 
    viewOffsetY=-5.06828)
session.viewports['Viewport: 1'].view.setValues(nearPlane=219.872, 
    farPlane=375.478, width=209.925, height=126.058, cameraPosition=(-49.4747, 
    214.548, 288.201), cameraUpVector=(0.0333202, 0.998186, -0.0501401), 
    cameraTarget=(-101.073, 306.205, -7.43127), viewOffsetX=18.7931, 
    viewOffsetY=-4.73625)
session.viewports['Viewport: 1'].view.setValues(nearPlane=217.753, 
    farPlane=377.597, width=235.29, height=141.289, viewOffsetX=17.6748, 
    viewOffsetY=-6.4338)
session.viewports['Viewport: 1'].view.setValues(nearPlane=227.628, 
    farPlane=367.557, width=245.96, height=147.696, cameraPosition=(-76.8675, 
    238.195, 294.711), cameraUpVector=(0.0785841, 0.988685, -0.127776), 
    cameraTarget=(-99.686, 306.324, -10.7384), viewOffsetX=18.4762, 
    viewOffsetY=-6.72554)
session.viewports['Viewport: 1'].view.setValues(nearPlane=200.357, 
    farPlane=392.75, width=216.493, height=130.002, cameraPosition=(-61.5633, 
    68.1291, 198.092), cameraUpVector=(-0.0132708, 0.857697, 0.513985), 
    cameraTarget=(-101.249, 308.194, -0.0380707), viewOffsetX=16.2627, 
    viewOffsetY=-5.9198)
session.viewports['Viewport: 1'].view.setValues(nearPlane=200.777, 
    farPlane=391.4, width=216.947, height=130.274, cameraPosition=(-39.3217, 
    3.33265, 67.767), cameraUpVector=(0.00603164, 0.512646, 0.858579), 
    cameraTarget=(-101.745, 305.318, 9.71603), viewOffsetX=16.2968, 
    viewOffsetY=-5.93221)
session.viewports['Viewport: 1'].view.setValues(nearPlane=191.184, 
    farPlane=401.491, width=206.582, height=124.05, cameraPosition=(55.283, 
    89.884, -173.969), cameraUpVector=(0.0993499, -0.311464, 0.94505), 
    cameraTarget=(-101.475, 283.371, 16.9542), viewOffsetX=15.5182, 
    viewOffsetY=-5.64878)
session.viewports['Viewport: 1'].view.setValues(nearPlane=235.58, 
    farPlane=359.212, width=254.553, height=152.856, cameraPosition=(-95.1445, 
    246.84, 295.341), cameraUpVector=(-0.0864937, 0.982254, -0.166424), 
    cameraTarget=(-100.537, 300.474, -13.781), viewOffsetX=19.1217, 
    viewOffsetY=-6.9605)
session.viewports['Viewport: 1'].view.setValues(nearPlane=215.685, 
    farPlane=379.139, width=233.056, height=139.947, cameraPosition=(-148.947, 
    393.046, 271.583), cameraUpVector=(0.150974, 0.791431, -0.592321), 
    cameraTarget=(-95.4471, 295.429, -21.7948), viewOffsetX=17.5069, 
    viewOffsetY=-6.37269)
#: The contents of viewport "Viewport: 1" have been copied to the clipboard.
session.viewports['Viewport: 1'].view.setValues(nearPlane=192.912, 
    farPlane=405.231, width=208.449, height=125.171, cameraPosition=(160.031, 
    122.307, 72.573), cameraUpVector=(-0.0990592, 0.700424, 0.706819), 
    cameraTarget=(-100.09, 289.302, 18.6119), viewOffsetX=15.6584, 
    viewOffsetY=-5.69983)
session.viewports['Viewport: 1'].view.setValues(nearPlane=202.556, 
    farPlane=396.001, width=218.869, height=131.428, cameraPosition=(-221.872, 
    441.046, 216.879), cameraUpVector=(0.217895, 0.653067, -0.725277), 
    cameraTarget=(-91.9237, 289.197, -25.024), viewOffsetX=16.4412, 
    viewOffsetY=-5.98477)
session.viewports['Viewport: 1'].view.setValues(nearPlane=222.998, 
    farPlane=375.478, width=240.957, height=144.692, cameraPosition=(13.05, 
    276.973, 286.527), cameraUpVector=(0.0849747, 0.951384, -0.296055), 
    cameraTarget=(-102.002, 306.819, -3.87611), viewOffsetX=18.1004, 
    viewOffsetY=-6.58875)
session.viewports['Viewport: 1'].view.setValues(nearPlane=194.504, 
    farPlane=403.816, width=210.168, height=126.204, cameraPosition=(105.971, 
    176.224, 208.743), cameraUpVector=(0.0890473, 0.995368, 0.0362486), 
    cameraTarget=(-99.8422, 306.911, 11.2001), viewOffsetX=15.7876, 
    viewOffsetY=-5.74686)
session.viewports['Viewport: 1'].view.setValues(nearPlane=229.747, 
    farPlane=367.113, width=248.25, height=149.072, cameraPosition=(-81.605, 
    230.185, 294.139), cameraUpVector=(0.28857, 0.951566, -0.106061), 
    cameraTarget=(-97.4209, 311.155, -8.60668), viewOffsetX=18.6482, 
    viewOffsetY=-6.78817)
session.viewports['Viewport: 1'].view.setValues(nearPlane=188.592, 
    farPlane=408.108, width=203.78, height=122.368, cameraPosition=(68.5487, 
    102.051, 181.724), cameraUpVector=(0.030019, 0.9058, 0.42264), 
    cameraTarget=(-102.573, 302.597, 11.5471), viewOffsetX=15.3077, 
    viewOffsetY=-5.57218)
session.viewports['Viewport: 1'].view.setValues(nearPlane=209.11, 
    farPlane=388.322, width=225.951, height=135.681, cameraPosition=(18.4956, 
    193.861, 267.656), cameraUpVector=(0.0204563, 0.99963, 0.0179196), 
    cameraTarget=(-103.153, 305.764, 0.933411), viewOffsetX=16.9731, 
    viewOffsetY=-6.17842)
session.viewports['Viewport: 1'].view.setValues(nearPlane=193.843, 
    farPlane=402.784, width=209.455, height=125.776, cameraPosition=(-331.366, 
    423.046, 89.2849), cameraUpVector=(0.637965, 0.722596, -0.266189), 
    cameraTarget=(-70.5726, 294.197, -28.386), viewOffsetX=15.7339, 
    viewOffsetY=-5.72735)
session.viewports['Viewport: 1'].view.setValues(nearPlane=196.986, 
    farPlane=399.278, width=212.851, height=127.815, cameraPosition=(-334.059, 
    418.882, -85.7764), cameraUpVector=(0.714575, 0.698033, -0.0461754), 
    cameraTarget=(-56.1784, 288.578, -20.4687), viewOffsetX=15.989, 
    viewOffsetY=-5.8202)
session.viewports['Viewport: 1'].view.setValues(nearPlane=213.744, 
    farPlane=380.849, width=230.959, height=138.689, cameraPosition=(-158.107, 
    191.968, 270.959), cameraUpVector=(0.345655, 0.931672, 0.111851), 
    cameraTarget=(-93.9968, 314.276, -10.8078), viewOffsetX=17.3492, 
    viewOffsetY=-6.31535)
session.viewports['Viewport: 1'].view.setValues(nearPlane=208.387, 
    farPlane=389.485, width=225.17, height=135.213, cameraPosition=(23.0275, 
    402.054, 260.067), cameraUpVector=(0.0992461, 0.713124, -0.693977), 
    cameraTarget=(-103.153, 303.623, -9.84344), viewOffsetX=16.9144, 
    viewOffsetY=-6.15707)
session.viewports['Viewport: 1'].view.setValues(nearPlane=189.405, 
    farPlane=406.188, width=204.659, height=122.896, cameraPosition=(-248.565, 
    97.7253, 153.4), cameraUpVector=(0.545004, 0.51522, 0.661452), 
    cameraTarget=(-85.1403, 319.926, 3.79857), viewOffsetX=15.3736, 
    viewOffsetY=-5.59621)
session.viewports['Viewport: 1'].view.setValues(nearPlane=235.78, 
    farPlane=360.176, width=254.769, height=152.987, cameraPosition=(-131.325, 
    326.003, 292.824), cameraUpVector=(-0.161147, 0.879669, -0.447453), 
    cameraTarget=(-102.511, 292.183, -17.7995), viewOffsetX=19.1378, 
    viewOffsetY=-6.96642)
session.viewports['Viewport: 1'].view.setValues(nearPlane=187.197, 
    farPlane=408.71, width=202.274, height=121.464, cameraPosition=(94.3019, 
    522.978, 74.7715), cameraUpVector=(-0.563108, 0.275851, -0.778983), 
    cameraTarget=(-108.222, 293.49, 5.63322), viewOffsetX=15.1944, 
    viewOffsetY=-5.53097)
session.viewports['Viewport: 1'].view.setValues(nearPlane=223.129, 
    farPlane=372.374, width=241.099, height=144.778, cameraPosition=(11.7054, 
    337.22, 281.833), cameraUpVector=(0.219556, 0.818994, -0.530137), 
    cameraTarget=(-101.922, 307.073, -9.09772), viewOffsetX=18.1109, 
    viewOffsetY=-6.59261)
session.viewports['Viewport: 1'].view.setValues(nearPlane=220.871, 
    farPlane=374.476, width=238.659, height=143.313, cameraPosition=(-52.3774, 
    228.762, 292.005), cameraUpVector=(0.541149, 0.819872, -0.186994), 
    cameraTarget=(-93.8566, 315.597, -6.65894), viewOffsetX=17.9276, 
    viewOffsetY=-6.52589)
session.viewports['Viewport: 1'].view.setValues(nearPlane=235.321, 
    farPlane=360.505, width=254.272, height=152.689, cameraPosition=(-90.6536, 
    351.872, 293.008), cameraUpVector=(0.494316, 0.745406, -0.447237), 
    cameraTarget=(-94.5708, 308.355, -17.7201), viewOffsetX=19.1005, 
    viewOffsetY=-6.95282)
session.viewports['Viewport: 1'].view.setValues(nearPlane=231.255, 
    farPlane=364.571, width=300.85, height=180.657, viewOffsetX=17.828, 
    viewOffsetY=-6.96191)
session.viewports['Viewport: 1'].view.setValues(nearPlane=197.742, 
    farPlane=401.178, width=257.251, height=154.477, cameraPosition=(-359.017, 
    368.526, 73.2402), cameraUpVector=(0.603893, 0.749261, 0.271886), 
    cameraTarget=(-66.1669, 306.149, -20.6164), viewOffsetX=15.2444, 
    viewOffsetY=-5.95301)
session.viewports['Viewport: 1'].view.setValues(nearPlane=203.005, 
    farPlane=396.503, width=264.097, height=158.588, cameraPosition=(-346.666, 
    359.207, -115.966), cameraUpVector=(0.269368, 0.748506, 0.605953), 
    cameraTarget=(-56.5533, 307.332, -8.24066), viewOffsetX=15.6501, 
    viewOffsetY=-6.11144)
session.viewports['Viewport: 1'].view.setValues(nearPlane=209.884, 
    farPlane=388.615, width=273.046, height=163.962, cameraPosition=(-59.0424, 
    150.517, 265.844), cameraUpVector=(0.280851, 0.949984, 0.136579), 
    cameraTarget=(-98.0941, 310.559, -1.21899), viewOffsetX=16.1804, 
    viewOffsetY=-6.31854)
session.viewports['Viewport: 1'].view.setValues(nearPlane=234.612, 
    farPlane=363.988, width=305.215, height=183.279, cameraPosition=(-135.306, 
    309.159, 294.861), cameraUpVector=(0.212025, 0.92057, -0.328017), 
    cameraTarget=(-96.974, 303.001, -16.5128), viewOffsetX=18.0867, 
    viewOffsetY=-7.06297)
session.viewports['Viewport: 1'].view.setValues(nearPlane=237.541, 
    farPlane=361.058, width=226.796, height=136.189, viewOffsetX=20.4515, 
    viewOffsetY=-4.41101)
session.viewports['Viewport: 1'].view.setValues(nearPlane=210.853, 
    farPlane=381.323, width=201.315, height=120.888, cameraPosition=(-370.651, 
    307.1, 50.4332), cameraUpVector=(0.343335, 0.931673, -0.118773), 
    cameraTarget=(-66.9086, 297.467, -27.7238), viewOffsetX=18.1538, 
    viewOffsetY=-3.91542)
session.viewports['Viewport: 1'].view.setValues(nearPlane=214.784, 
    farPlane=378.325, width=205.069, height=123.142, cameraPosition=(-301.298, 
    304.716, 197.166), cameraUpVector=(0.276192, 0.936275, -0.217041), 
    cameraTarget=(-83.5606, 299.833, -28.7265), viewOffsetX=18.4923, 
    viewOffsetY=-3.98842)
session.viewports['Viewport: 1'].view.setValues(nearPlane=213.862, 
    farPlane=378.312, width=204.189, height=122.613, cameraPosition=(-372.479, 
    305.041, 39.1658), cameraUpVector=(0.34061, 0.933869, -0.108965), 
    cameraTarget=(-65.9647, 297.461, -27.5593), viewOffsetX=18.4129, 
    viewOffsetY=-3.9713)
session.viewports['Viewport: 1'].view.setValues(nearPlane=225.502, 
    farPlane=367.369, width=215.302, height=129.286, cameraPosition=(-197.029, 
    283.228, 273.443), cameraUpVector=(0.173165, 0.957095, -0.232343), 
    cameraTarget=(-95.8767, 302.25, -22.9814), viewOffsetX=19.415, 
    viewOffsetY=-4.18744)
session.viewports['Viewport: 1'].view.setValues(nearPlane=212.212, 
    farPlane=378.675, width=202.613, height=121.666, cameraPosition=(-164.919, 
    187.673, 265.183), cameraUpVector=(-0.00683302, 0.999055, 0.0429166), 
    cameraTarget=(-99.4606, 304.995, -18.3863), viewOffsetX=18.2707, 
    viewOffsetY=-3.94065)
session.viewports['Viewport: 1'].view.setValues(nearPlane=220.17, 
    farPlane=372.997, width=210.211, height=126.228, cameraPosition=(-242.991, 
    312.428, 247.602), cameraUpVector=(0.265656, 0.924141, -0.274572), 
    cameraTarget=(-90.5977, 300.384, -26.4274), viewOffsetX=18.9558, 
    viewOffsetY=-4.08842)
#: The contents of viewport "Viewport: 1" have been copied to the clipboard.
session.viewports['Viewport: 1'].view.setValues(nearPlane=198.001, 
    farPlane=394.337, width=189.045, height=113.519, cameraPosition=(80.023, 
    183.304, 228.628), cameraUpVector=(-0.186246, 0.965479, 0.182108), 
    cameraTarget=(-108.082, 297.54, 4.95929), viewOffsetX=17.0471, 
    viewOffsetY=-3.67675)
session.viewports['Viewport: 1'].view.setValues(nearPlane=220.926, 
    farPlane=371.819, width=210.933, height=126.663, cameraPosition=(-25.8866, 
    230.888, 287.496), cameraUpVector=(-0.112148, 0.989342, -0.0928723), 
    cameraTarget=(-106.508, 299.747, -7.83412), viewOffsetX=19.0209, 
    viewOffsetY=-4.10246)
import os
os.chdir(
    r"E:\101-WorkShop\simulation\solidwork\fractal cut\abaqus-simulation\single\drag-2000sqrt2")
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON)
mdb.models['Model-1'].loads['Load-1'].setValues(cf1=-2000.0, cf2=-2000.0, 
    distributionType=UNIFORM, field='')
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF)
mdb.Job(name='F-2000sqrt2', model='Model-1', description='', type=ANALYSIS, 
    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
    scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=1, 
    numGPUs=0)
mdb.jobs['F-2000sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-2000sqrt2.inp" has been submitted for analysis.
#: Job F-2000sqrt2: Analysis Input File Processor completed successfully.
#: Error in job F-2000sqrt2: Time increment required is less than the minimum specified
#: Error in job F-2000sqrt2: THE ANALYSIS HAS BEEN TERMINATED DUE TO PREVIOUS ERRORS. ALL OUTPUT REQUESTS HAVE BEEN WRITTEN FOR THE LAST CONVERGED INCREMENT.
#: Job F-2000sqrt2: Abaqus/Standard aborted due to errors.
#: Error in job F-2000sqrt2: Abaqus/Standard Analysis exited with an error - Please see the  message file for possible error messages if the file exists.
#: Job F-2000sqrt2 aborted due to errors.
session.viewports['Viewport: 1'].setValues(
    displayedObject=session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-200sqrt2/F-200sqrt2.odb'])
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       3
#: Number of Node Sets:          7
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=64)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=65)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=60)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=34)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=39)
session.viewports['Viewport: 1'].view.setValues(nearPlane=228.053, 
    farPlane=399.621, width=168.614, height=101.251, cameraPosition=(207.728, 
    334.246, 122.228), cameraUpVector=(-0.539584, 0.829747, 0.142721), 
    cameraTarget=(-77.6586, 296.177, -2.53937))
session.viewports['Viewport: 1'].view.setValues(nearPlane=227.733, 
    farPlane=395.454, width=168.378, height=101.109, cameraPosition=(132.458, 
    270.662, 229.118), cameraUpVector=(-0.485675, 0.870014, 0.0848218), 
    cameraTarget=(-77.6708, 296.167, -2.522))
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=44)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=55)
session.viewports['Viewport: 1'].view.setValues(nearPlane=258.96, 
    farPlane=362.022, width=191.466, height=114.974, cameraPosition=(-103.523, 
    334.628, 307.252), cameraUpVector=(-0.271509, 0.841954, -0.466258), 
    cameraTarget=(-76.4029, 295.823, -2.94181))
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=58)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=59)
session.viewports['Viewport: 1'].view.setValues(nearPlane=229.626, 
    farPlane=394.569, width=169.778, height=101.95, cameraPosition=(114.7, 
    270.844, 243.41), cameraUpVector=(-0.570895, 0.813731, 0.109183), 
    cameraTarget=(-78.8606, 296.541, -2.22279))
session.viewports['Viewport: 1'].view.setValues(nearPlane=247.262, 
    farPlane=373.456, width=182.817, height=109.78, cameraPosition=(-9.34756, 
    265.57, 302.028), cameraUpVector=(-0.552737, 0.822385, -0.134779), 
    cameraTarget=(-78.1896, 296.569, -2.53987))
session.viewports['Viewport: 1'].view.setValues(nearPlane=221.875, 
    farPlane=408.056, width=164.046, height=98.5085, cameraPosition=(29.0754, 
    464.164, 239.064), cameraUpVector=(-0.531152, 0.58733, -0.610672), 
    cameraTarget=(-78.6139, 294.376, -1.84457))
session.viewports['Viewport: 1'].view.setValues(nearPlane=252.081, 
    farPlane=371.711, width=186.38, height=111.92, cameraPosition=(4.07814, 
    290.541, 301.04), cameraUpVector=(-0.441588, 0.87092, -0.215636), 
    cameraTarget=(-78.7075, 293.726, -1.61256))
session.viewports['Viewport: 1'].view.setValues(nearPlane=236.02, 
    farPlane=383.506, width=174.505, height=104.789, cameraPosition=(-223.504, 
    280.511, 275.882), cameraUpVector=(-0.299492, 0.815061, -0.495964), 
    cameraTarget=(-77.3278, 293.787, -1.46004))
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=62)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=66)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=70)
session.viewports['Viewport: 1'].view.setValues(nearPlane=238.667, 
    farPlane=381.758, width=176.463, height=105.964, cameraPosition=(-29.3173, 
    210.61, 296.263), cameraUpVector=(-0.692011, 0.721283, -0.0295267), 
    cameraTarget=(-80.0353, 294.762, -1.74422))
session.viewports['Viewport: 1'].view.setValues(nearPlane=246.187, 
    farPlane=377.975, width=182.023, height=109.303, cameraPosition=(-56.9988, 
    372.789, 300.797), cameraUpVector=(-0.640182, 0.615246, -0.460043), 
    cameraTarget=(-79.7164, 292.893, -1.79646))
session.viewports['Viewport: 1'].view.setValues(nearPlane=245.726, 
    farPlane=378.436, width=181.682, height=109.098, cameraPosition=(-56.9988, 
    372.789, 300.797), cameraUpVector=(-0.649447, 0.607526, -0.457309), 
    cameraTarget=(-79.7164, 292.893, -1.79646))
session.viewports['Viewport: 1'].view.setValues(nearPlane=232.027, 
    farPlane=388.899, width=171.554, height=103.016, cameraPosition=(-9.29274, 
    193.526, 287.063), cameraUpVector=(-0.848242, 0.528805, 0.0291832), 
    cameraTarget=(-79.9772, 293.873, -1.72137))
session.viewports['Viewport: 1'].view.setValues(nearPlane=212.858, 
    farPlane=407.357, width=157.381, height=94.5053, cameraPosition=(46.0846, 
    118.043, 225.258), cameraUpVector=(-0.838755, 0.425711, 0.339501), 
    cameraTarget=(-80.5701, 294.681, -1.05966))
session.viewports['Viewport: 1'].view.setValues(nearPlane=244.267, 
    farPlane=377.834, width=180.604, height=108.45, cameraPosition=(-120.52, 
    365.737, 300.094), cameraUpVector=(-0.684591, 0.470648, -0.55662), 
    cameraTarget=(-78.5935, 291.742, -1.94754))
session.viewports['Viewport: 1'].view.setValues(nearPlane=216.268, 
    farPlane=403.542, width=159.902, height=96.0189, cameraPosition=(-2.81721, 
    88.8887, 223.269), cameraUpVector=(-0.812569, 0.511379, 0.279685), 
    cameraTarget=(-79.6289, 294.177, -1.2717))
session.viewports['Viewport: 1'].view.setValues(nearPlane=217.753, 
    farPlane=400.172, width=161, height=96.6781, cameraPosition=(-32.732, 
    9.1977, 120.492), cameraUpVector=(-0.720854, 0.464994, 0.513955), 
    cameraTarget=(-79.2542, 295.175, 0.0155182))
session.viewports['Viewport: 1'].view.setValues(nearPlane=234.574, 
    farPlane=383.035, width=173.437, height=104.146, cameraPosition=(-191.092, 
    334.183, 286.307), cameraUpVector=(-0.437905, 0.638601, -0.632794), 
    cameraTarget=(-76.7817, 290.101, -2.57336))
session.viewports['Viewport: 1'].view.setValues(nearPlane=243.701, 
    farPlane=374.897, width=180.185, height=108.199, cameraPosition=(-115.744, 
    228.053, 302.085), cameraUpVector=(-0.651139, 0.707081, -0.27578), 
    cameraTarget=(-77.9972, 291.813, -2.82788))
session.viewports['Viewport: 1'].view.setValues(nearPlane=236.264, 
    farPlane=382.833, width=174.686, height=104.897, cameraPosition=(-64.5579, 
    158.187, 280.562), cameraUpVector=(-0.721913, 0.691971, -0.00420076), 
    cameraTarget=(-78.7398, 292.827, -2.51561))
session.viewports['Viewport: 1'].view.setValues(nearPlane=230.27, 
    farPlane=390.097, width=170.254, height=102.236, cameraPosition=(-107.209, 
    453.964, 262.926), cameraUpVector=(-0.263243, 0.570044, -0.778301), 
    cameraTarget=(-78.1559, 288.778, -2.27414))
session.viewports['Viewport: 1'].view.setValues(nearPlane=211.929, 
    farPlane=403.941, width=156.693, height=94.0928, cameraPosition=(-305.758, 
    370.843, 196.054), cameraUpVector=(0.153395, 0.740104, -0.654764), 
    cameraTarget=(-75.8499, 289.743, -1.49747))
session.viewports['Viewport: 1'].view.setValues(nearPlane=241.609, 
    farPlane=377.872, width=178.637, height=107.27, cameraPosition=(-0.587151, 
    333.118, 296.708), cameraUpVector=(-0.26472, 0.876711, -0.401624), 
    cameraTarget=(-81.6482, 290.46, -3.4099))
session.viewports['Viewport: 1'].view.setValues(nearPlane=237.746, 
    farPlane=380.329, width=175.781, height=105.555, cameraPosition=(-50.5953, 
    189.637, 291.599), cameraUpVector=(-0.293292, 0.955986, 0.00847071), 
    cameraTarget=(-80.9951, 292.334, -3.34318))
session.viewports['Viewport: 1'].view.setValues(nearPlane=222.863, 
    farPlane=397.664, width=164.777, height=98.9471, cameraPosition=(223.513, 
    322.768, -44.0012), cameraUpVector=(-0.336529, 0.76092, 0.554751), 
    cameraTarget=(-85.207, 290.288, 1.81358))
session.viewports['Viewport: 1'].view.setValues(nearPlane=246.837, 
    farPlane=374.296, width=182.503, height=109.591, cameraPosition=(-148.555, 
    263.481, 303.062), cameraUpVector=(-0.175732, 0.93853, -0.297119), 
    cameraTarget=(-80.9822, 290.961, -2.12732))
session.viewports['Viewport: 1'].view.setValues(nearPlane=247.158, 
    farPlane=374.493, width=182.74, height=109.733, cameraPosition=(-98.7295, 
    359.809, 303.298), cameraUpVector=(-0.270538, 0.797015, -0.539978), 
    cameraTarget=(-81.4987, 289.962, -2.12977))
session.viewports['Viewport: 1'].view.setValues(nearPlane=242.063, 
    farPlane=378.801, width=178.973, height=107.471, cameraPosition=(-64.11, 
    194.334, 295.766), cameraUpVector=(-0.26863, 0.963017, -0.0209004), 
    cameraTarget=(-81.8284, 291.538, -2.05804))
#: The contents of viewport "Viewport: 1" have been copied to the clipboard.
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON, mesh=OFF)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=OFF)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
mdb.models['Model-1'].materials['Al'].elastic.setValues(table=((6000.0, 0.01), 
    ))
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.jobs['F-2000sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-2000sqrt2.inp" has been submitted for analysis.
#: Job F-2000sqrt2: Analysis Input File Processor completed successfully.
#: Error in job F-2000sqrt2: Time increment required is less than the minimum specified
#: Error in job F-2000sqrt2: THE ANALYSIS HAS BEEN TERMINATED DUE TO PREVIOUS ERRORS. ALL OUTPUT REQUESTS HAVE BEEN WRITTEN FOR THE LAST CONVERGED INCREMENT.
#: Job F-2000sqrt2: Abaqus/Standard aborted due to errors.
#: Error in job F-2000sqrt2: Abaqus/Standard Analysis exited with an error - Please see the  message file for possible error messages if the file exists.
#: Job F-2000sqrt2 aborted due to errors.
session.viewports['Viewport: 1'].setValues(
    displayedObject=session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb'])
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       3
#: Number of Node Sets:          7
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=10)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=14)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=15)
session.viewports['Viewport: 1'].view.setValues(nearPlane=232.891, 
    farPlane=394.725, width=172.191, height=103.399, cameraPosition=(102.936, 
    478.124, 178.934), cameraUpVector=(-0.95817, 0.25896, 0.12186), 
    cameraTarget=(-78.2282, 296.959, -2.23111))
session.viewports['Viewport: 1'].view.setValues(nearPlane=238.704, 
    farPlane=388.005, width=176.489, height=105.98, cameraPosition=(93.6715, 
    140.405, 208.508), cameraUpVector=(-0.96361, -0.0328689, 0.265283), 
    cameraTarget=(-78.2288, 296.936, -2.2291))
session.viewports['Viewport: 1'].view.setValues(nearPlane=287.571, 
    farPlane=334.04, width=212.62, height=127.676, cameraPosition=(-82.8262, 
    280.706, 310.969), cameraUpVector=(-0.923394, -0.144827, -0.355485), 
    cameraTarget=(-77.9856, 296.743, -2.37025))
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=9)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=11)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=14)
session.viewports['Viewport: 1'].view.setValues(nearPlane=237.042, 
    farPlane=385.222, width=175.26, height=105.242, cameraPosition=(-70.7882, 
    -11.5374, 40.6702), cameraUpVector=(-0.562364, 0.219344, -0.797267), 
    cameraTarget=(-78.101, 299.546, 0.221771))
session.viewports['Viewport: 1'].view.setValues(nearPlane=225.267, 
    farPlane=394.915, width=166.554, height=100.014, cameraPosition=(-182.185, 
    4.09039, 15.1033), cameraUpVector=(-0.402851, 0.458108, -0.792369), 
    cameraTarget=(-77.1503, 299.413, 0.439964))
session.viewports['Viewport: 1'].view.setValues(nearPlane=234.195, 
    farPlane=387.028, width=173.155, height=103.978, cameraPosition=(-123.882, 
    -10.0154, -21.3358), cameraUpVector=(-0.475532, 0.462248, -0.748462), 
    cameraTarget=(-77.8453, 299.581, 0.874374))
session.viewports['Viewport: 1'].view.setValues(nearPlane=280.112, 
    farPlane=340.508, width=207.104, height=124.364, cameraPosition=(-119.245, 
    318.001, 307.797), cameraUpVector=(-0.889074, -0.0865609, -0.449505), 
    cameraTarget=(-77.8927, 296.227, -2.49097))
session.viewports['Viewport: 1'].view.setValues(nearPlane=279.911, 
    farPlane=341.754, width=206.955, height=124.275, cameraPosition=(-56.4604, 
    323.808, 309.275), cameraUpVector=(-0.963961, -0.0286118, -0.264499), 
    cameraTarget=(-78.5963, 296.162, -2.50754))
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=15)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=11)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=7)
session.viewports['Viewport: 1'].view.setValues(nearPlane=281.954, 
    farPlane=340.291, width=236.893, height=142.252, viewOffsetX=8.58545, 
    viewOffsetY=6.46159)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=15)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=13)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=10)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=5)
#: The contents of viewport "Viewport: 1" have been copied to the clipboard.
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
mdb.models['Model-1'].materials['Al'].elastic.setValues(table=((600000.0, 
    0.01), ))
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.jobs['F-2000sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-2000sqrt2.inp" has been submitted for analysis.
#: Job F-2000sqrt2: Analysis Input File Processor completed successfully.
#: Error in job F-2000sqrt2: Time increment required is less than the minimum specified
#: Error in job F-2000sqrt2: THE ANALYSIS HAS BEEN TERMINATED DUE TO PREVIOUS ERRORS. ALL OUTPUT REQUESTS HAVE BEEN WRITTEN FOR THE LAST CONVERGED INCREMENT.
#: Job F-2000sqrt2: Abaqus/Standard aborted due to errors.
#: Error in job F-2000sqrt2: Abaqus/Standard Analysis exited with an error - Please see the  message file for possible error messages if the file exists.
#: Job F-2000sqrt2 aborted due to errors.
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       3
#: Number of Node Sets:          7
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=66)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=66)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=68)
#: The contents of viewport "Viewport: 1" have been copied to the clipboard.
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=61)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=63)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1')
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=34)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=40)
session.viewports['Viewport: 1'].view.setValues(nearPlane=233.716, 
    farPlane=390.157, width=172.801, height=103.765, cameraPosition=(112.09, 
    293.52, 247.674), cameraUpVector=(-0.36746, 0.920878, -0.130216), 
    cameraTarget=(-78.1756, 295.898, -1.83603))
session.viewports['Viewport: 1'].view.setValues(nearPlane=215.856, 
    farPlane=403.807, width=159.596, height=95.8356, cameraPosition=(60.2365, 
    170.19, 249.948), cameraUpVector=(-0.215855, 0.95884, 0.184477), 
    cameraTarget=(-77.8681, 296.629, -1.84953))
session.viewports['Viewport: 1'].view.setValues(nearPlane=209.821, 
    farPlane=410.191, width=155.134, height=93.1561, cameraPosition=(137.43, 
    153.043, 175.182), cameraUpVector=(-0.152499, 0.924788, 0.348585), 
    cameraTarget=(-78.8534, 296.848, -0.895252))
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=47)
session.viewports['Viewport: 1'].odbDisplay.setFrame(step='Step-1', frame=58)
session.viewports['Viewport: 1'].view.setValues(nearPlane=245.924, 
    farPlane=380.204, width=181.827, height=109.185, cameraPosition=(-36.3368, 
    376.639, 298.368), cameraUpVector=(-0.398455, 0.767538, -0.502115), 
    cameraTarget=(-77.5253, 295.139, -1.83678))
session.viewports['Viewport: 1'].view.setValues(nearPlane=226.809, 
    farPlane=401.126, width=167.694, height=100.698, cameraPosition=(-131.536, 
    479.574, 246.108), cameraUpVector=(-0.405637, 0.410468, -0.816685), 
    cameraTarget=(-77.3057, 294.902, -1.71626))
session.viewports['Viewport: 1'].view.setValues(nearPlane=233.613, 
    farPlane=395.793, width=172.725, height=103.719, cameraPosition=(111.359, 
    318.441, 248.008), cameraUpVector=(-0.425557, 0.886549, -0.181473), 
    cameraTarget=(-77.1652, 294.809, -1.71517))
session.viewports['Viewport: 1'].view.setValues(nearPlane=233.847, 
    farPlane=395.559, width=174.315, height=104.675, viewOffsetX=0.566072, 
    viewOffsetY=1.51719)
session.viewports['Viewport: 1'].view.setValues(nearPlane=247.328, 
    farPlane=374.462, width=184.364, height=110.709, cameraPosition=(-145.455, 
    320.304, 303.701), cameraUpVector=(-0.253189, 0.84597, -0.469287), 
    cameraTarget=(-77.8126, 294.803, -1.64429), viewOffsetX=0.598705, 
    viewOffsetY=1.60465)
session.viewports['Viewport: 1'].view.setValues(nearPlane=236.52, 
    farPlane=385.952, width=176.307, height=105.871, cameraPosition=(-175.567, 
    347.554, 292.004), cameraUpVector=(-0.224254, 0.787682, -0.573818), 
    cameraTarget=(-77.5026, 294.579, -1.31971), viewOffsetX=0.572541, 
    viewOffsetY=1.53453)
session.viewports['Viewport: 1'].view.setValues(nearPlane=253.129, 
    farPlane=371.619, width=188.688, height=113.306, cameraPosition=(-61.7412, 
    346.599, 307.507), cameraUpVector=(-0.446514, 0.776105, -0.445294), 
    cameraTarget=(-78.222, 294.592, -1.49995), viewOffsetX=0.612747, 
    viewOffsetY=1.64229)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
mdb.models['Model-1'].materials['Al'].elastic.setValues(table=((600000.0, 0.3), 
    ))
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF, mesh=ON)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=ON)
#: 
#: Part instance:  LEVEL1-13,  Number of regions verified:  1
#:   Number of elements :  5505,   Analysis errors:  0 (0%),  Analysis warnings:  0 (0%)
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=ON, bcs=ON, 
    predefinedFields=ON, connectors=ON)
session.viewports['Viewport: 1'].view.setValues(nearPlane=703.074, 
    farPlane=851.772, width=297.209, height=178.471, viewOffsetX=106.333, 
    viewOffsetY=117.901)
session.viewports['Viewport: 1'].view.setValues(nearPlane=634.639, 
    farPlane=856.002, width=268.279, height=161.099, cameraPosition=(648.781, 
    319.892, 197.407), cameraUpVector=(-0.345524, 0.749169, -0.565119), 
    cameraTarget=(-133.62, 240.888, 103.94), viewOffsetX=95.9826, 
    viewOffsetY=106.425)
session.viewports['Viewport: 1'].view.setValues(nearPlane=653.507, 
    farPlane=837.135, width=80.1434, height=48.1253, viewOffsetX=57.0014, 
    viewOffsetY=127.822)
session.viewports['Viewport: 1'].view.setValues(nearPlane=649.52, 
    farPlane=904.394, width=79.6545, height=47.8317, cameraPosition=(557.728, 
    506.903, 347.547), cameraUpVector=(-0.369601, 0.619318, -0.692705), 
    cameraTarget=(-141.28, 233.513, 95.0112), viewOffsetX=56.6536, 
    viewOffsetY=127.042)
session.viewports['Viewport: 1'].view.setValues(nearPlane=633.87, 
    farPlane=920.043, width=272.366, height=163.553, viewOffsetX=60.9218, 
    viewOffsetY=120.766)
session.viewports['Viewport: 1'].view.setValues(nearPlane=599.825, 
    farPlane=802.635, width=257.738, height=154.769, cameraPosition=(26.0378, 
    -20.6826, 677.002), cameraUpVector=(0.316715, 0.940913, -0.119898), 
    cameraTarget=(-191.998, 242.214, -37.4729), viewOffsetX=57.6497, 
    viewOffsetY=114.279)
session.viewports['Viewport: 1'].view.setValues(nearPlane=646.496, 
    farPlane=787.121, width=277.792, height=166.811, cameraPosition=(-236.68, 
    63.8563, 701.753), cameraUpVector=(0.514709, 0.851935, -0.0963404), 
    cameraTarget=(-185.462, 256.081, -64.7687), viewOffsetX=62.1353, 
    viewOffsetY=123.171)
session.viewports['Viewport: 1'].view.setValues(nearPlane=652.996, 
    farPlane=780.622, width=195.952, height=117.668, viewOffsetX=66.3561, 
    viewOffsetY=111.166)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON, mesh=OFF)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=OFF)
mdb.models['Model-1'].materials['Al'].elastic.setValues(table=((600000000.0, 
    0.3), ))
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(loads=OFF, bcs=OFF, 
    predefinedFields=OFF, connectors=OFF)
mdb.jobs['F-2000sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-2000sqrt2.inp" has been submitted for analysis.
#: Job F-2000sqrt2: Analysis Input File Processor completed successfully.
#: Job F-2000sqrt2: Abaqus/Standard completed successfully.
#: Job F-2000sqrt2 completed successfully. 
session.viewports['Viewport: 1'].setValues(
    displayedObject=session.odbs['E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb'])
o3 = session.openOdb(
    name='E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb')
#: Model: E:/101-WorkShop/simulation/solidwork/fractal cut/abaqus-simulation/single/drag-2000sqrt2/F-2000sqrt2.odb
#: Number of Assemblies:         1
#: Number of Assembly instances: 0
#: Number of Part instances:     1
#: Number of Meshes:             1
#: Number of Element Sets:       3
#: Number of Node Sets:          7
#: Number of Steps:              1
session.viewports['Viewport: 1'].setValues(displayedObject=o3)
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].view.setValues(nearPlane=247.068, 
    farPlane=379.61, width=182.673, height=109.693, cameraPosition=(83.1705, 
    170.338, 235.278), cameraUpVector=(-0.943065, 0.165344, 0.288599), 
    cameraTarget=(-78.2345, 296.861, -2.21324))
session.viewports['Viewport: 1'].view.setValues(nearPlane=269.304, 
    farPlane=355.257, width=199.113, height=119.565, cameraPosition=(21.0267, 
    244.008, 290.7), cameraUpVector=(-0.987037, 0.160382, 0.00597671), 
    cameraTarget=(-78.1458, 296.756, -2.29232))
session.viewports['Viewport: 1'].view.setValues(nearPlane=280.31, 
    farPlane=342.284, width=207.25, height=124.451, cameraPosition=(-67.7472, 
    233.171, 304.744), cameraUpVector=(-0.848475, 0.48465, -0.212613), 
    cameraTarget=(-77.7177, 296.808, -2.36006))
session.animationController.setValues(animationType=TIME_HISTORY, viewports=(
    'Viewport: 1', ))
session.animationController.play(duration=UNLIMITED)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
mdb.models['Model-1'].materials['Al'].elastic.setValues(table=((600000.0, 0.3), 
    ))
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
mdb.jobs['F-2000sqrt2'].submit(consistencyChecking=OFF)
#: The job input file "F-2000sqrt2.inp" has been submitted for analysis.
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF, mesh=ON)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=ON)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
#: Job F-2000sqrt2: Analysis Input File Processor completed successfully.
elemType1 = mesh.ElemType(elemCode=SC8R, elemLibrary=STANDARD, 
    secondOrderAccuracy=OFF, hourglassControl=DEFAULT)
elemType2 = mesh.ElemType(elemCode=SC6R, elemLibrary=STANDARD, 
    secondOrderAccuracy=OFF)
elemType3 = mesh.ElemType(elemCode=UNKNOWN_TET, elemLibrary=STANDARD)
p = mdb.models['Model-1'].parts['LEVEL1-13']
c = p.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
pickedRegions =(cells, )
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, 
    elemType3))
#: Error in job F-2000sqrt2: Time increment required is less than the minimum specified
#: Error in job F-2000sqrt2: THE ANALYSIS HAS BEEN TERMINATED DUE TO PREVIOUS ERRORS. ALL OUTPUT REQUESTS HAVE BEEN WRITTEN FOR THE LAST CONVERGED INCREMENT.
#: Job F-2000sqrt2: Abaqus/Standard aborted due to errors.
#: Error in job F-2000sqrt2: Abaqus/Standard Analysis exited with an error - Please see the  message file for possible error messages if the file exists.
#: Job F-2000sqrt2 aborted due to errors.
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON, mesh=OFF)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=OFF)
mdb.models['Model-1'].PEGSection(name='Section-3', material='Al', 
    thickness=1.0, wedgeAngle1=0.0, wedgeAngle2=0.0)
mdb.models['Model-1'].sections.changeKey(fromName='Section-3', 
    toName='S-plane')
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p = mdb.models['Model-1'].parts['LEVEL1-13']
region = p.sets['whole part']
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.SectionAssignment(region=region, sectionName='Solid', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)
session.viewports['Viewport: 1'].view.setValues(nearPlane=286.096, 
    farPlane=334.127, width=188.574, height=113.237, viewOffsetX=13.8084, 
    viewOffsetY=-4.93373)
mdb.models['Model-1'].sections['Solid'].setValues(material='Al', thickness=1.0)
del mdb.models['Model-1'].parts['LEVEL1-13'].sectionAssignments[0]
del mdb.models['Model-1'].parts['LEVEL1-13'].sectionAssignments[0]
p = mdb.models['Model-1'].parts['LEVEL1-13']
region = p.sets['whole part']
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.SectionAssignment(region=region, sectionName='Solid', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF, mesh=ON)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=ON)
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON, 
    engineeringFeatures=ON, mesh=OFF)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=OFF)
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=OFF, 
    engineeringFeatures=OFF)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=ON)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=200.0)
g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
s1.setPrimaryObject(option=STANDALONE)
s1.undo()
#* Nothing to undo.
s1.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']
session.viewports['Viewport: 1'].view.setValues(nearPlane=2.64621, 
    farPlane=4.46576, width=4.26209, height=2.55934, viewOffsetX=0.289051, 
    viewOffsetY=0.15786)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.regenerate()
#: The contents of viewport "Viewport: 1" have been copied to the clipboard.
p = mdb.models['Model-1'].parts['LEVEL1-13']
p.regenerate()
session.viewports['Viewport: 1'].view.setValues(nearPlane=280.969, 
    farPlane=339.254, width=243.367, height=146.14, viewOffsetX=-6.01817, 
    viewOffsetY=9.1119)
session.viewports['Viewport: 1'].view.setValues(nearPlane=269.618, 
    farPlane=353.701, width=233.536, height=140.236, cameraPosition=(-135.795, 
    237.579, 301.497), cameraUpVector=(-0.391075, 0.88747, -0.24384), 
    cameraTarget=(-80.3253, 296.233, 2.07826), viewOffsetX=-5.77505, 
    viewOffsetY=8.74381)
session.viewports['Viewport: 1'].view.setValues(nearPlane=267.192, 
    farPlane=356.02, width=231.434, height=138.974, cameraPosition=(-48.7088, 
    391.87, 295.599), cameraUpVector=(-0.0681068, 0.787866, -0.612069), 
    cameraTarget=(-83.4876, 293.26, 3.64789), viewOffsetX=-5.72308, 
    viewOffsetY=8.66512)
session.viewports['Viewport: 1'].view.setValues(nearPlane=269.604, 
    farPlane=353.609, width=194.754, height=116.948, viewOffsetX=2.38889, 
    viewOffsetY=9.78379)
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
    sheetSize=200.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
session.viewports['Viewport: 1'].view.setValues(nearPlane=165.257, 
    farPlane=211.866, width=279.478, height=167.824, cameraPosition=(28.4113, 
    22.8335, 188.562), cameraTarget=(28.4113, 22.8335, 0))
s.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']
session.viewports['Viewport: 1'].view.setValues(nearPlane=2.81545, 
    farPlane=4.68748, width=2.98614, height=1.79315, viewOffsetX=0.227769, 
    viewOffsetY=0.126309)
p1 = mdb.models['Model-1'].parts['LEVEL1-13']
session.viewports['Viewport: 1'].setValues(displayedObject=p1)
session.viewports['Viewport: 1'].view.setValues(nearPlane=273.178, 
    farPlane=351.14, width=162.564, height=97.618, viewOffsetX=1.46606, 
    viewOffsetY=1.77353)
session.viewports['Viewport: 1'].partDisplay.setValues(mesh=ON)
session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
    meshTechnique=ON)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=OFF)
