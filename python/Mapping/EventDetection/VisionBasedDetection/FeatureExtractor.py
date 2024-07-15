class Object:
    """
    Object class used for SfM
    
    """
    def __init__(self,featureType,scene_obj_ids,scenes,id):
        self.featureType = featureType
        self.scene_obj_ids = [scene_obj_ids]
        self.scenes = [scenes]
        self.state_idxs = []
        self.id = id # Unique object ID

    def addSceneObjID(self,scene_obj_id):
        self.scene_obj_ids.append(scene_obj_id)
    
    def addScene(self,scene):
        self.scenes.append(scene)

    def addStateIdxs(self,idxs):
        self.state_idxs.extend(idxs)


class FeatureAnalyzer:
    """
    Dummy class for resolving pkl file errors.
    """
    def __init__(self,featureType):
        self.featureType = featureType
