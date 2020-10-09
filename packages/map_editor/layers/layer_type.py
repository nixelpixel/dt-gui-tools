from enum import Enum
from collections import namedtuple

from classes.mapTile import MapTile
from classes.mapObjects import MapBaseObject, SignObject, GroundAprilTagObject, WatchTowerObject, RegionObject, ActorObject, DecorationObject

LayerInfo = namedtuple('LayerInfo', ['type', 'obj_type', 'obj_class'])


class LayerType(Enum):
    TILES = LayerInfo('0-tiles', None, MapTile)
    TRAFFIC_SIGNS = LayerInfo('1-signs', 'sign', SignObject)
    GROUND_APRILTAG = LayerInfo('2-groundtags', 'apriltag', GroundAprilTagObject)
    WATCHTOWERS = LayerInfo('3-watchtowers', 'watchtower', WatchTowerObject)
    REGIONS = LayerInfo('4-regions', None, RegionObject)
    ACTORS = LayerInfo('5-actors', 'actor', ActorObject)
    DECORATIONS = LayerInfo('6-decorations', 'decoration', DecorationObject)
    ITEMS = LayerInfo('objects', 'object', MapBaseObject)

    def __str__(self):
        return self.value.type

    def get_obj_class(self):
        return self.value.obj_class
