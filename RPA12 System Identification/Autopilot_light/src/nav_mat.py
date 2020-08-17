import numpy as np


def clear_mat(x=20):
    # Add this to shipmat function
    ship_update_mat = np.zeros((x))

    ship_update_mat[0]  = 0.0    # Time
    ship_update_mat[1]  = 0      # ID
    ship_update_mat[2]  = 51.9   # Lats
    ship_update_mat[3]  = 4.4    # Longs
    ship_update_mat[4]  = 0      # Heading
    ship_update_mat[5]  = 0      # Speed
    ship_update_mat[6]  = 10.0
    ship_update_mat[7]  = 0.0001
    ship_update_mat[8]  = 0.0001
    ship_update_mat[9]  = 0.0001
    ship_update_mat[10] = 0.0001
    ship_update_mat[11] = 0.0001
    ship_update_mat[12] = 0.0001
    ship_update_mat[13] = 0.0001
    ship_update_mat[14] = 15.0
    ship_update_mat[15] = 15.0
    ship_update_mat[16] = 15.0
    ship_update_mat[17] = 15.0

    return (ship_update_mat)


def ship_update(ships_mat, ship_update_mat):
    # print(ship_update_mat)
    # print(f"nav_mat : ship_update_mat {ship_update_mat[2:4]}")

    lookatmat(ships_mat)

    # Look for all existing records of the ships mmsi
    existing_ship_records = np.where(ships_mat[:-1, :, 1] == ship_update_mat[1])  # ignore the last time slice
    # print(existing_ship_records)
    # Save how many records there are
    if len(existing_ship_records[0]) > 0:
        number_of_records = len(existing_ship_records[1])
    else:
        number_of_records = 0

    if number_of_records > 0:  # vessel_id:
        # print(f'Old rec found for  {mmsi}')

        itteration_int = existing_ship_records[1][-1]  # Seperate out the record number
        itteration_time = existing_ship_records[0][-1]  # Seperate out the time slice

        # The fist -1 is to get it to the right format the second is to ignore the last time slice reserved for predictions
        if itteration_time == ships_mat.shape[0] - 1 - 1:
            # Roll all values
            ships_mat[existing_ship_records[0], itteration_int, :] = np.roll(
                ships_mat[existing_ship_records[0], itteration_int, :], -1, axis=0)
            ships_mat[itteration_time, itteration_int, :] = ship_update_mat

            # Add ship dimentions from a previous itteration time
            ships_mat[itteration_time, itteration_int, 14:18] = ships_mat[itteration_time - 1, itteration_int, 14:18]
            return (ships_mat)

        else:
            # Here we use the incremented ship record id to load ship information in a new time slice
            new_ship_record = existing_ship_records[0][-1] + 1
            ships_mat[new_ship_record, itteration_int, :] = ship_update_mat

            # Add dimentions from a previous itteration time to save time from database lookup
            ships_mat[new_ship_record, itteration_int, 14:18] = ships_mat[existing_ship_records[0][-1], itteration_int,
                                                                14:18]
            return (ships_mat)

    else:  # Add new ship (This could be done smarter)

        array_of_empty = np.where(ships_mat[0, :, 1] == 0.0)
        if len(array_of_empty[0]) > 0:
            qty = array_of_empty[0][0]

        elif len(array_of_empty[0]) == 0:  # Delete old ship records
            print(f'nmea_receive: there is no empty slots {len(array_of_empty[0])} deleting oldest record')
            # U, C, I = np.unique(ships_mat[0, :, 1], return_counts=True, return_index=True)
            smallest_time = np.min(ships_mat[0, 1:, 0])
            array_to_delete = np.where(ships_mat[0, :, 0] < smallest_time)
            print(
                f'nmea_receive:  This is the smallest time  {smallest_time} and this is the array to delete {array_to_delete}')
            for time_slice in np.arange(ships_mat.shape[0] - 1):
                ships_mat[time_slice, array_to_delete, :] = clear_mat()

            return (ships_mat)

        ships_mat[0, qty, :] = ship_update_mat
        return (ships_mat)

    return (ships_mat)


def ego_update(ships_mat, ship_update_mat):
    print("Write code to handel Updating variables of ego vehicle")


def lookatmat(ships_mat):
    num_ships0 = np.where(ships_mat[0, :, 1] != 0)[0]
    num_ships1 = np.where(ships_mat[1, :, 1] != 0)[0]
    num_ships2 = np.where(ships_mat[2, :, 1] != 0)[0]
    num_ships3 = np.where(ships_mat[3, :, 1] != 0)[0]