xacro --inorder $(rospack find oberon7_description)/robots/oberon7_standalone.xacro > $(rospack find oberon7_description)/models/oberon7/model.urdf
gz sdf -p $(rospack find oberon7_description)/models/oberon7/model.urdf > $(rospack find oberon7_description)/models/oberon7/model.sdf
