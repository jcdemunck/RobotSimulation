from collections import defaultdict

from XdockParams import N_DOCK, N_BUFFER_STORE
from Position import Position
from SimulationConfig import get_output_dock, destination_from_dock, prio_from_dock

class BufferStoreManager:
    def __init__(self):
        self.loc_dp_dict = dict([((d,b), (destination_from_dock(d), prio_from_dock(d))) for d in range(N_DOCK) for b in range(N_BUFFER_STORE)])
        self.buffer_dict  = defaultdict(list)
        self.__update_buffer_dict()

    def __update_buffer_dict(self):
        self.buffer_dict = defaultdict(list)
        for dock in range(N_DOCK):
            for buffer in range(N_BUFFER_STORE):
                self.buffer_dict[self.loc_dp_dict[dock,buffer]].append((dock,buffer))

    def get_buffer_list(self, dest, prio):
        return self.buffer_dict[dest, prio]

    def find_available_buffer_store(self, floor_plan, dest, prio):
        dock_store_list = self.buffer_dict[dest, prio]
        for (dock, buffer) in dock_store_list:
            if floor_plan.buffer_stores[dock, buffer].get_first_available_store()>=0:
                return floor_plan.buffer_stores[dock, buffer]

        # First search for a free store in the default output dock
        dock = get_output_dock(dest, prio)
        for buffer in range(N_BUFFER_STORE):
            if floor_plan.buffer_stores[dock, buffer].is_store_unused():
                self.loc_dp_dict[dock, buffer] = (dest, prio)
                self.__update_buffer_dict()
                return floor_plan.buffer_stores[dock, buffer]

        # Search globally
        docks = [dock + pm * (d + 1) for d in range(N_DOCK) for pm in [-1, 1] if 0<=dock + pm * (d + 1)<N_DOCK]
        for dock in docks:
            for buffer in range(N_BUFFER_STORE - 1, -1, -1):
                if floor_plan.buffer_stores[dock, buffer].is_store_unused():
                    self.loc_dp_dict[dock, buffer] = (dest, prio)
                    self.__update_buffer_dict()
                    return floor_plan.buffer_stores[dock, buffer]

        print("ERROR: BufferStoreManager.find_available_buffer_store(). Cannot find unused buffer. ")

    def choose_store(self, floor_plan, roll_container):
        dest = roll_container.dest
        prio = roll_container.prio
        dock = get_output_dock(dest, prio)
        lane = floor_plan.get_best_available_lane(dock, output=True)

        if lane<0:
            buffer_store = self.find_available_buffer_store(floor_plan, dest, prio)
            if buffer_store is None:
                print("ERROR: SimulationConfig.choose_store(). buffer overflow. prio = ", prio, "dest = ", dest)

            store        = buffer_store.buffer
            dock         = buffer_store.dock
            row          = buffer_store.get_first_available_store()
            pos_store    = Position(floor_plan, dock, buffer_store=store, row=row, col=0)
            buffer_store.reserve_store(row)
        else:
            pos_store = Position(floor_plan, dock, buffer_lane=lane)
            floor_plan.buffer_lanes[(dock, lane)].reserve_store()

        return pos_store

