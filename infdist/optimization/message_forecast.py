from heapq import heappush, heappop


class MessageForecast:
    def __init__(self, message_context):
        self.type_forecasts = {
            _type.message_forecast_cls(**_type.message_forecast_kwargs)
            for _type in message_context.message_types
        }

    def register(self, message):
        print(f"Registering {message}")

    def estimate_t_end(self):
        return max([
            _type.estimate_t_end()
            for _type in self.type_forecasts
        ])

    def message_generator(self, t_start):
        heap = []
        for generator in [
            _type.message_generator(t_start)
            for _type in self.type_forecasts
        ]:
            message = next(generator)
            heappush(heap, (message.t_gen, message, generator))

        while heap:
            t_gen, message, generator = heappop(heap)
            yield message
            try:
                new_message = next(generator)
                heappush(heap, (new_message.t_gen, new_message, generator))
            except StopIteration:
                pass


class BaseTypeForecast:
    def register_message(self, message):
        raise NotImplementedError()

    def forecast_generator(self, n):
        raise NotImplementedError()


class FullKnowledgeTypeForecast(BaseTypeForecast):
    def __init__(self, messages):
        self.messages = messages

    def estimate_t_end(self):
        return self.messages.t_end

    def message_generator(self, t_start):
        return iter(self.messages.filter(gen_after=t_start).all())
