
Model humanModel("gait_model.osim");

Model afoModel("afo_model.osim");

List<Connectors>& attachments = afoModel.getUnsatisfiedConnectors();

Body& tibia = humanModel.get<Body>("tibia");
attachments.get<Body>("cuff_attachment").bind<WeldJoint>(tibia);

Body& calcn = humanModel.get<Body>("calcn");
attachments.get<Body>("footplate_attachment").bind<SliderJoint>(calcn, OffSetFrame(Vec3(1, 2, 3)));
