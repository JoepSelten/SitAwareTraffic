import sqlite3

con = sqlite3.connect("test.db")
con.enable_load_extension(True)

con.execute('SELECT load_extension("mod_spatialite")')   
con.execute('SELECT InitSpatialMetaData(1);')  

cur = con.cursor()

cur.execute('''CREATE TABLE my_line(id INTEGER PRIMARY KEY)''')
cur.execute('SELECT AddGeometryColumn("my_line","geom" , 4326, "LINESTRING", 2)')
con.commit()

polygon_wkt = 'POLYGON ((11 50,11 51,12 51,12 50,11 50))'

XA = 11
YA = 52
XB = 12
YB = 49

line_wkt = 'LINESTRING({0} {1}, {2} {3})'.format(XA, YA, XB, YB)

cur.execute("""INSERT INTO my_line VALUES (?,GeomFromText(?, 4326))""", (1, line_wkt))

con.commit()

cursor = con.execute('''
    SELECT astext(st_intersection(geom, GeomFromText(?, 4326))) from my_line
    WHERE st_intersects(geom, GeomFromText(?, 4326))''', (polygon_wkt, polygon_wkt))

for item in cursor:
    print(item)


#cur.execute("CREATE TABLE movie(title, year, score)")

#res = cur.execute("SELECT name FROM sqlite_master")
#print(res.fetchone())