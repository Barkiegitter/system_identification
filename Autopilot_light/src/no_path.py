
class no_path(object):
    def __init__(self, settings):
        self.static_path = settings["NAV_PATH"]['force_static_path']
    def GeneratePaths(self, recieved, for_send, ships_mat):
        return (for_send, ships_mat)