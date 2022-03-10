from collections import Counter
from os import listdir
from os.path import join
import logging
from typing import Optional

import yaml
from dataclasses import dataclass

logger = logging.getLogger('root')
CONFIG_PATH = './doc/apriltagsDB.yaml'


def get_id_by_type(type_of_obj: str) -> Optional[int]:
    with open(CONFIG_PATH) as file:
        content = yaml.safe_load(file)
        for tag in content:
            if tag['traffic_sign_type'] == type_of_obj:
                return int(tag['tag_id'])


@dataclass
class TagStore:
    id: int
    type: str


def count_elements(data): return Counter(data)


def get_list_dir(dir_path):
    try:
        entries = listdir(dir_path)
        return entries
    except FileNotFoundError as e:
        logger.warning(e)
        return []


def get_list_dir_with_path(dir_path): return [(filename, join(dir_path, filename)) for filename in
                                              get_list_dir(dir_path)]


def get_available_translations(lang_dir_path): return {filename[len('lang_'):-len('.qm')]: path for filename, path in
                                                       get_list_dir_with_path(lang_dir_path)}


def get_degree_for_orientation(orientation: str) -> int:
    degrees: dict = {
        'S': 180,
        'E': 270,
        'N': 0,
        'W': 90,
    }
    return degrees[orientation]


def get_orientation_for_degree(degree: int) -> str:
    degrees: dict = {
         0: 'N',
         270: 'E',
         180: 'S',
         90: 'W'
    }
    return degrees[degree]


SIGNS_ALIASES = {
    "T-intersection": "sign_T_intersect",
    "oneway-right": "sign_1_way_right",
    "oneway-left": "sign_1_way_left",
    "duck-crossing": "sign_duck_crossing",
    "stop": "sign_stop",
    "yield": "sign_yield",
    "no-left-turn": "sign_no_left_turn",
    "t-light-ahead": "sign_t_light_ahead",
    "pedestrian": "sign_pedestrian",
    "no-right-turn": "sign_no_right_turn",
    "parking": "sign_parking",
    "right-T-intersect": "sign_right_T_intersect",
    "left-T-intersect": "sign_left_T_intersect",
    "4-way-intersect": "sign_4_way_intersect",
    "do-not-enter": "sign_do_not_enter",
}


def get_canonical_sign_name(name_of_sign: str) -> str:
    if name_of_sign in SIGNS_ALIASES:
        return SIGNS_ALIASES[name_of_sign]
    return name_of_sign

if __name__ == '__main__':
    print(get_id_by_type('stop'))