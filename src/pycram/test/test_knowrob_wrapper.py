import logging

from pycram.knowrob.knowrob_wrapper import KnowRob

if __name__ == '__main__':
    root = logging.getLogger()
    root.setLevel(logging.INFO)

    knowrob = KnowRob()
    assert knowrob.load_owl("package://knowrob/owl/maps/iai_room_v1.owl", "map", "http://knowrob.org/kb/v1/IAI-Kitchen.owl#")
    knowrob.once("tf_logger_enable")
