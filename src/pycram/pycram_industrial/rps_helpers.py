import logging


def assert_ok(res: dict) -> dict:
    if not res["success"]:
        logging.error("RPS control plugin command returned failure")
        logging.error(res)
        raise AssertionError()
    return res

