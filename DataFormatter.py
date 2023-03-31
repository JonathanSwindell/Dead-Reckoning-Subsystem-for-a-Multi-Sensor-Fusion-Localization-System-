import csv

def format_data(bno_data, gps_data, out_file):
    output_csv = open(out_file,"w", newline='')
    print("Time, Time (ms), System Calibration, Gyro Cal, Accel Cal, Mag cal, Pos x, Pos y, Posz, mag x, mag y, mag z, gyro x, gyro y, gyro z, quat x, quat y, quat z, quat w, linAcc x, linAcc y, linAcc z, Euler x, Euler y, Euler z, AbsAcc x, AbsAcc y, AbsAcc z, Lat, Long", file=output_csv)
    out_csv_writer = csv.writer(output_csv, delimiter=',')
    more_gps_rows = True
    with open(bno_data) as bno_csv:
        with open(gps_data) as gps_csv:
            bno_csv_reader = csv.reader(bno_csv, delimiter=',')
            gps_csv_reader = csv.reader(gps_csv, delimiter=',')
            #gps_prev_row = next(gps_csv_reader)
            gps_row = next(gps_csv_reader)
            for row in bno_csv_reader:
                bno_time_ms = int(row[1])
                if (bno_time_ms >= 99970):
                    print("hi")
                    print("asdasd")
                gps_time_ms = int(gps_row[1])
                if more_gps_rows:
                    while gps_time_ms < bno_time_ms or ((bno_time_ms < ((gps_time_ms+30)%100000)) and (((gps_time_ms+30)%100000) != (gps_time_ms+30))):
                        #gps_prev_row = gps_row
                        try:
                            gps_row = next(gps_csv_reader)
                            gps_time_ms = int(gps_row[1])
                        except:
                            # Out of GPS rows
                            more_gps_rows = False
                            break

                if (more_gps_rows):
                    # Not interpolating right now >:)
                    lat = gps_row[2]
                    long = gps_row[3]
                    row.append(lat)
                    row.append(long)
                    out_csv_writer.writerow(row)

    output_csv.close()



if __name__ == "__main__":
    format_data("BNO_2nd_loop.csv","GPS_2nd_loop.csv", "2nd_loop_lot_combined.csv")
    #format_data("BNO_parking_lot.csv","GPS_parking_lot.csv", "parking_lot_combined.csv")