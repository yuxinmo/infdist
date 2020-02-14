class BaseNetwork:
    """ This class defines an interface for network implementations """

    def deserialize(self, native_message):
        """
        Converts messages native to network format to
        simulator.models.Message
        """
        raise NotImplementedError()

    # The following function is not used
    # def serialize(self, message):
    #     """
    #     Converts objects of type simulator.models.Message
    #     to messages native to the network
    #     """
    #     raise NotImplementedError()

    def add_message_received_callback(self, callback, node_id):
        """
        Whenever a message is received on node with id node_id, the given
        callback should be called.

        Node_id is needed for simulated networks, when multiple devices
        are handled with a single network class.
        """
        raise NotImplementedError()

    def send(self, native_message):
        """
        Send message to the network
        """
        raise NotImplementedError()
