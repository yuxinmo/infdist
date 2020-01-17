from .models import MessageSet
from optimization.missions import simulate_sending_messages_with_latency


class DynamicMessageTree:
    def __init__(self):
        self.past_messages = MessageSet(0)
        self.future_messages = MessageSet(0)
        self.latency = 0.001

    def update_future(self, future_messages):
        self.future_messages = future_messages
        simulate_sending_messages_with_latency(
            self.future_messages,
            self.latency,
        )

    def progress_time(self, t):
        self.future_messages = self.future_messages.filter(
            received_after=t
        )

    def register_message(self, message):
        """
        TODO: remove registered messages from future
        then in decide function we no longer have to skip message we decide on
        """
        self.past_messages.append(message)
        print(self)

    def decide(self, message):
        print("&&&&", message)
        for m in self.future_messages.all():
            if (
                m.sender == message.sender and
                m.data_type_name == message.data_type_name
            ):
                # skip the message we decide on
                continue

        return True

    def __str__(self):
        return (
            "####\n"
            "Past:\n {}\n"
            "Future:\n {}\n"
            "----\n"
        ).format(
            self.past_messages,
            self.future_messages,
        )
