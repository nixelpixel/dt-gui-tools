from layers.layer_type import LayerType
#from classes.mapObjects import MapBaseObject, SignObject, CityObject, WatchTowerObject, GroundAprilTagObject


LAYER_TYPE_WITH_OBJECTS = (LayerType.TRAFFIC_SIGNS, LayerType.GROUND_APRILTAG, LayerType.WATCHTOWERS, LayerType.ACTORS, LayerType.DECORATIONS)


def get_class_by_object_type(object_type):
    """
    Get map object class from classes.mapObjects for object_type
    If object_type doesn't exist, return MapBaseObject
    :param object_type: type of object
    :return: class of map object
    """
    for layer_type_info in list(LayerType):
        layer_type, obj_type, obj_class = layer_type_info.value
        if object_type == obj_type:
            return obj_class
    return LayerType.ITEMS.get_obj_class()


def get_class_by_layer_type(layer_type):
    """
    Get map object class from classes.mapObjects for layer_type
    :param layer_type: LayerType
    :return: class of map object
    """
    return layer_type.get_obj_class()


def get_layer_type_by_object_type(object_type):
    """
    Get layer type by object type (help know what layer type need for this object_type)
    If object_type doesn't exist, return LayerType.ITEMS
    :param object_type: type of object
    :return: LayerType
    """
    for layer_type_info in list(LayerType):
        layer_type, obj_type, obj_class = layer_type_info.value
        if object_type == obj_type:
            return layer_type_info
    return LayerType.ITEMS


def get_layer_type_by_value(type_value):
    for layer_type in LayerType: 
        if type_value == layer_type.value:
            return layer_type
