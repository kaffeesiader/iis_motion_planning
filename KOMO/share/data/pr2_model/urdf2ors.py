#!/usr/bin/python

from lxml import etree

#inFile = "pr2-with-ft-sensors.urdf" #misses the mimic entries!?
inFile = "z.urdf"
xmlData = etree.parse(inFile)

links = xmlData.findall("/link")
for link in links:
    name = link.attrib['name']
    print 'body %s {' % name,

    elem = link.find("inertial/mass")
    if elem is not None:
        print ' mass=%s' % elem.attrib['value'],

    elem = link.find("inertial/inertia")
    if elem is not None:
        print ' inertiaTensor=[%s %s %s %s %s %s]' % (
            elem.attrib['ixx'],
            elem.attrib['ixy'],
            elem.attrib['ixz'],
            elem.attrib['iyy'],
            elem.attrib['iyz'],
            elem.attrib['izz']),

    elem = link.find("collision/origin")
    if elem is not None:
        print ' rel=<T t(%s) E(%s)>' % (elem.attrib['xyz'],
                                         elem.attrib['rpy']),

    elem = link.find("collision/geometry/box")
    if elem is not None:
        print ' type=0 size=[%s 0]' % elem.attrib['size'],

    elem = link.find("collision/geometry/sphere")
    if elem is not None:
        print ' type=1 size=[0 0 0 %s]' % elem.attrib['radius'],

    elem = link.find("collision/geometry/cylinder")
    if elem is not None:
        print ' type=2 size=[0 0 %s %s]' % (elem.attrib['length'],
                                             elem.attrib['radius']),

    elem = link.find("collision/geometry/mesh")
    if elem is not None:
        meshfile = elem.attrib['filename']
        meshfile = meshfile.replace("package://pr2_description/meshes/",
                                    "")
        print ' type=3 mesh=\'%s\'' % meshfile,

    print '}\n',

joints = xmlData.findall("/joint")
for joint in joints:
    name = joint.attrib['name']
    if joint.find("child") is not None:
        print 'joint %s (%s %s) {' % (name,
                                      joint.find("parent").attrib['link'],
                                      joint.find("child").attrib['link']),

        # figure out joint type
        att = joint.attrib.get('type')
        if att in ["revolute", "continuous"]:
            print ' type=0',
        if att == "prismatic":
            print ' type=3',
        if att == "fixed":
            print ' type=10',

        elem = joint.find("mimic")
        if elem is not None:
            print ' mimic=%s' % elem.attrib['joint'],

        elem = joint.find("axis")
        if elem is not None:
            print ' axis=[%s]' % elem.attrib['xyz'],

        elem = joint.find("origin")
        if elem is not None:
            att = elem.attrib.get('rpy')
            if att is not None:
                print ' A=<T t(%s) E(%s)>' % (elem.attrib['xyz'], att),
            else:
                print ' A=<T t(%s)>' % (elem.attrib['xyz']),

        elem = joint.find("limit")
        if elem is not None:
            lo = elem.attrib.get('lower')
            up = elem.attrib.get('upper')
            eff = elem.attrib.get('effort')
            vel = elem.attrib.get('velocity')
            if lo is not None:
                print ' limits=[%s %s]' % (lo, up),
            if vel is not None:
                print ' ctrl_limits=[%s %s]' % (vel, eff),

        print '}\n',

#print(etree.tostring(links[22]))
#print(etree.tostring(joints[0]))
