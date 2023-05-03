import csv
import math

def longlat_to_meters(long0, lat0, long1, lat1): # X is N/S, Y is E/W. Z is altitude
    a = 6_378_137 # Semi-major axis for WGS-84
    b = 6_356_732.3142 #Semi-minor axis
    ab = a * b
    a2 = a * a
    b2 = b * b
    theta = (long1 * (math.pi / 180)) - (long0 * (math.pi / 180))  # Change in longitude in rad. (Longitude final - longitude initial)
    phi0 = lat0 * (math.pi / 180)
    phi1 = lat1 * (math.pi / 180)
    dY = theta * ab * math.sqrt(1/(b2 + a2*math.tan(phi0))) # dX = r*theta, r = radius of equatorial line at given latitude

    dX_r = (ab / math.sqrt(a2*math.pow(math.sin(phi0),2) + b2*math.pow(math.cos(phi0),2))) #Find radius of ellipse at given latitude
    dX = dX_r * math.sqrt(2-2*math.cos(phi1-phi0)) #Law of cosines
    if (lat1 < lat0):
        dX = dX * -1
    return dX, dY

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



def format_data2(combined_data, out_file):
    output_csv = open(out_file, "w", newline='')
    out_csv_writer = csv.writer(output_csv, delimiter=',')
    with open(combined_data) as bno_csv:
        bno_csv_reader = csv.reader(bno_csv, delimiter=',')
        header = next(bno_csv_reader)
        row1 = next(bno_csv_reader)
        lat0 = float(row1[28])
        lon0 = float(row1[29])
        counter = 0
        for row in bno_csv_reader:
            counter = counter + 1
            #print(row)
            lat=float(row[28])
            lon=float(row[29])
            gps_dx, gps_dy = longlat_to_meters(lon0,lat0, lon, lat)
            out_row = [row[6], row[7], gps_dx, gps_dy]
            if (counter % 100 == 0):
                out_csv_writer.writerow(out_row)
    output_csv.close()
if __name__ == "__main__":
    format_data2("April3CombinedCampusLoop.csv", "Testing_bigloop.csv")
    #format_data("April3BNOCampusLoop.csv","April3GPSCampusLoop.csv", "April3CombinedCampusLoop.csv")
    #format_data("BNO_parking_lot.csv","GPS_parking_lot.csv", "parking_lot_combined.csv")