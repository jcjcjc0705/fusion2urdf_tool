# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku
"""

import adsk, re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils


class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit

    def make_joint_xml(self):
        joint = Element('joint')
        joint.attrib = {'name': self.name, 'type': self.type}

        origin = SubElement(joint, 'origin')
        origin.attrib = {
            'xyz': ' '.join([str(_) for _ in self.xyz]),
            'rpy': '0 0 0'
        }

        parent = SubElement(joint, 'parent')
        parent.attrib = {'link': self.parent}

        child = SubElement(joint, 'child')
        child.attrib = {'link': self.child}

        if self.type in ['revolute', 'continuous', 'prismatic']:
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz': ' '.join([str(_) for _ in self.axis])}

        if self.type in ['revolute', 'prismatic']:
            limit = SubElement(joint, 'limit')
            limit.attrib = {
                'upper': str(self.upper_limit),
                'lower': str(self.lower_limit),
                'effort': '100',
                'velocity': '100'
            }

        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        tran = Element('transmission')
        tran.attrib = {'name': self.name + '_tran'}

        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'

        joint = SubElement(tran, 'joint')
        joint.attrib = {'name': self.name}
        hw_joint = SubElement(joint, 'hardwareInterface')
        hw_joint.text = 'hardware_interface/EffortJointInterface'

        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name': self.name + '_actr'}
        hw_act = SubElement(actuator, 'hardwareInterface')
        hw_act.text = 'hardware_interface/EffortJointInterface'

        reduction = SubElement(actuator, 'mechanicalReduction')
        reduction.text = '1'

        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])


def make_joints_dict(root, msg):

    joint_type_list = [
        'fixed', 'revolute', 'prismatic',
        'Cylinderical', 'PinSlot', 'Planner', 'Ball'
    ]

    joints_dict = {}

    for joint in root.joints:
        joint_dict = {}

        # ---- joint type ----
        try:
            joint_type = joint_type_list[joint.jointMotion.jointType]
        except:
            msg = f'Unsupported joint type: "{joint.name}"'
            break

        joint_dict['type'] = joint_type
        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0

        # ---- revolute ----
        if joint_type == 'revolute':
            joint_dict['axis'] = [
                round(i, 6)
                for i in joint.jointMotion.rotationAxisVector.asArray()
            ]

            rot_limits = joint.jointMotion.rotationLimits
            max_enabled = rot_limits.isMaximumValueEnabled
            min_enabled = rot_limits.isMinimumValueEnabled

            # URDF 的零位 = 匯出當下的 CAD 姿態，而 Fusion 的 limit 是相對於
            # 「建立 joint 當下」的姿態。兩者差一個目前的關節角度，不扣掉的話
            # 匯出的 limit 會整個偏移。
            try:
                rot_offset = joint.jointMotion.rotationValue
            except:
                rot_offset = 0.0

            if max_enabled and min_enabled:
                joint_dict['upper_limit'] = round(
                    rot_limits.maximumValue - rot_offset, 6)
                joint_dict['lower_limit'] = round(
                    rot_limits.minimumValue - rot_offset, 6)
            elif max_enabled:
                msg = joint.name + ' is missing lower limit.'
                break
            elif min_enabled:
                msg = joint.name + ' is missing upper limit.'
                break
            else:
                joint_dict['type'] = 'continuous'

        # ---- prismatic ----
        elif joint_type == 'prismatic':
            joint_dict['axis'] = [
                round(i, 6)
                for i in joint.jointMotion.slideDirectionVector.asArray()
            ]

            slide_limits = joint.jointMotion.slideLimits
            max_enabled = slide_limits.isMaximumValueEnabled
            min_enabled = slide_limits.isMinimumValueEnabled

            # 同 revolute：扣掉匯出姿態的位移，讓 limit 相對於 URDF 的零位
            try:
                slide_offset = joint.jointMotion.slideValue
            except:
                slide_offset = 0.0

            if max_enabled and min_enabled:
                joint_dict['upper_limit'] = round(
                    (slide_limits.maximumValue - slide_offset) / 100, 6)
                joint_dict['lower_limit'] = round(
                    (slide_limits.minimumValue - slide_offset) / 100, 6)
            elif max_enabled:
                msg = joint.name + ' is missing lower limit.'
                break
            elif min_enabled:
                msg = joint.name + ' is missing upper limit.'
                break

        # ---- SAFETY CHECK (CRITICAL) ----
        if joint.occurrenceOne is None or joint.occurrenceTwo is None:
            msg = (
                f'Invalid joint detected: "{joint.name}"\n\n'
                'This joint is detected by Fusion API but is not fully visible in the UI.\n'
                'It is likely a residual joint created by design history '
                '(e.g. Rigid / As-Built Joint) that was deleted or hidden later.\n\n'
                'Suggested actions:\n'
                '• Check As-Built Joints / Rigid Groups in the Browser\n'
                '• Roll back the timeline and remove the joint\n'
                '• Or save, close, and reopen the design to clear history\n\n'
                'This exporter only supports Component ↔ Component joints.\n'
                'Do NOT connect joints to Joint Origin / Ground / Root.'
            )
            break

        # ---- parent / child ----
        if joint.occurrenceTwo.component.name == 'base_link':
            joint_dict['parent'] = 'base_link'
        else:
            joint_dict['parent'] = re.sub('[ :()]', '_', joint.occurrenceTwo.name)

        joint_dict['child'] = re.sub('[ :()]', '_', joint.occurrenceOne.name)

        # ---- transform helpers ----
        def trans(M, a):
            ex = [M[0], M[4], M[8]]
            ey = [M[1], M[5], M[9]]
            ez = [M[2], M[6], M[10]]
            oo = [M[3], M[7], M[11]]
            return [
                a[0]*ex[i] + a[1]*ey[i] + a[2]*ez[i] + oo[i]
                for i in range(3)
            ]

        def allclose(v1, v2, tol=1e-6):
            return max(abs(a - b) for a, b in zip(v1, v2)) < tol

        # ---- joint origin ----
        try:
            xyz_from_one = joint.geometryOrOriginOne.origin.asArray()
            xyz_from_two = joint.geometryOrOriginTwo.origin.asArray()
            xyz_of_one = joint.occurrenceOne.transform.translation.asArray()
            M_two = joint.occurrenceTwo.transform.asArray()

            if allclose(xyz_from_two, xyz_from_one) or allclose(xyz_from_two, xyz_of_one):
                xyz_joint = xyz_from_two
            else:
                xyz_joint = trans(M_two, xyz_from_two)

            joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_joint]

        except:
            try:
                if isinstance(joint.geometryOrOriginTwo, adsk.fusion.JointOrigin):
                    data = joint.geometryOrOriginTwo.geometry.origin.asArray()
                else:
                    data = joint.geometryOrOriginTwo.origin.asArray()
                joint_dict['xyz'] = [round(i / 100.0, 6) for i in data]
            except:
                msg = joint.name + " doesn't have a valid joint origin."
                break

        joints_dict[joint.name] = joint_dict

    return joints_dict, msg
