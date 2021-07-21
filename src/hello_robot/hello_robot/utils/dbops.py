from datetime import datetime, timezone
import sqlite3
from sqlite3 import Error
from hello_robot.settings import DB_FILE
from hello_robot.settings import VELOCITYLOG, PROCESSING


def create_connection(db_file):
    """ create a database connection to the SQLite database
        specified by db_file
    :param db_file: database file
    :return: Connection object or None
    """
    conn = None
    try:
        conn = sqlite3.connect(db_file)
    except Error as e:
        print(e)

    return conn


def save_velocity_input(conn, lvelocity, avelocity):
    ts = datetime.now(tz=timezone.utc).isoformat(sep='T', timespec='seconds')
    
    query = f"""
        INSERT INTO {VELOCITYLOG}(linear, angular, timestamp)
        VALUES(
            {lvelocity},
            {avelocity},
            '{ts}'
        )
    """
    
    cur = conn.cursor()
    cur.execute(query)
    conn.commit()
    lastrowid = cur.lastrowid
    cur.close()
    return lastrowid

def process_velocity_input(conn, logid, lvelocity, avelocity):
    
    query = f"""
        INSERT INTO {PROCESSING}(logid, linear, angular)
        VALUES(
            {logid},
            {lvelocity},
            {avelocity}
        )
    """
    
    cur = conn.cursor()
    cur.execute(query)
    conn.commit()
    lastrowid = cur.lastrowid
    cur.close()
    return lastrowid


def get_min_logid(conn):
    logid = 0
    query = f"""
        SELECT MIN(logid) FROM {PROCESSING}
    """
    cur = conn.cursor()
    cur.execute(query)
    row = cur.fetchone()
    cur.close()
    
    if row[0]:
        logid = row[0]
    return logid


def get_command(conn, logid):
    
    query = f"""
        SELECT logid, linear, angular
        FROM {PROCESSING}
        WHERE logid = {logid}
    """
    cur = conn.cursor()
    cur.execute(query)
    command = cur.fetchone()
    cur.close()
    print(command)
    return command


def delete_command(conn, logid):
    
    query = f"""
        DELETE FROM {PROCESSING}
        WHERE logid = {logid}
    """
    cur = conn.cursor()
    cur.execute(query)
    conn.commit()
    cur.close()
    
      
def delete_all(conn):
    
    query = f"""
        DELETE FROM {PROCESSING}
    """    
    cur = conn.cursor()
    cur.execute(query)
    conn.commit()
    cur.close()
    
    
    
if __name__ == '__main__':
    DB_FILE = '/home/ros2/ROS2/robot_ws/data/robot.sqlite3'
    conn = create_connection(DB_FILE)  
       
    with conn:
        id = get_min_logid(conn)
        print(id)
        
    if conn:
        conn.close
    
    
    
    