from FloorPlan import N_DOCK, N_LANE, N_BUFFER_STORE, N_COMP_X, N_COMP_Y


class Position:
    def __init__(self, floor_plan, dock, buffer_lane=-1, parking=-1, store=-1, row=-1, col=-1):
        self.dock        = -1
        self.buffer_lane = -1
        self.parking     = -1
        self.store       = -1
        self.row         = -1
        self.col         = -1
        self.w           =  0.
        self.h           =  0.

        if dock<0 or N_DOCK<=dock:
            print("ERROR: Position.__init__(). dock = ", dock)
            return
        self.dock = dock

        if buffer_lane>=0:
            if buffer_lane>N_LANE:
                print("ERROR: Position.__init__(). buffer_lane = ", buffer_lane)
                return
            self.buffer_lane = buffer_lane
            self.w, self.h   = floor_plan.buffer_lanes[(dock, buffer_lane)].get_grid_coords()

        elif parking>=0:
            if parking>=floor_plan.parkings[dock].n_parc_spot:
                print("ERROR: Position.__init__(). parking = ", parking)
                return
            self.parking   = parking
            self.w, self.h = floor_plan.parkings[dock].get_grid_coords(park=parking)

        elif store>=0:
            if store>=N_BUFFER_STORE:
                print("ERROR: Position.__init__(). store = ", store)
                return
            if row<0 or N_COMP_Y<=row or col<0 or N_COMP_X<=col:
                print("ERROR: Position.__init__(). row, col = ", row, col)
                return

            self.store     = store
            self.w, self.h = floor_plan.buffer_stores[(dock, store)].get_grid_coords(row=row, col=col)
        else:
            print("ERROR: Position.__init__(). Invalid arguments ")

