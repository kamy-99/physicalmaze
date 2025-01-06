import os
import time
import serial
import sqlite3

from flask import Flask, redirect, render_template, request, session
from flask_session import Session
from werkzeug.security import check_password_hash, generate_password_hash
from functools import wraps

def create_app():
    app = Flask(__name__)
    app.config["SESSION_PERMANENT"] = False # no reason to make it true
    app.secret_key = os.environ.get("FLASK_SECRET_KEY") # for some reason it needed a secret key

    init_db()
    
    return app

def get_db_conn():
    conn = sqlite3.connect("robot.db")
    conn.row_factory = sqlite3.Row
    return conn

def init_db(): # instead of robot.db having the tables by default i have decided to create them when the website opens (i just didn't know how to put tables in the db)
        db = sqlite3.connect("robot.db")
        db.execute('''CREATE TABLE IF NOT EXISTS users (
                    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
                    username TEXT NOT NULL,
                    hash TEXT NOT NULL)''')
        
        db.execute('''CREATE UNIQUE INDEX IF NOT EXISTS username ON users (username)''')
        
        db.execute('''CREATE TABLE IF NOT EXISTS robot (
                    slave_num INTEGER PRIMARY KEY NOT NULL,
                    speed INTEGER NOT NULL,
                    lrotation INTEGER NOT NULL,
                    rrotation INTEGER NOT NULL,
                    action TEXT NOT NULL,
                    created_at DATETIME DEFAULT CURRENT_TIMESTAMP)''')
        
        db.commit()
        db.close()

app = create_app()

if __name__ == "__main__":
    app.run(debug=True) # for debugging

#need SQL, SQL table users:name-hash, table robot1-2-3:speed-LRotations-RRotations-action-time refresh every second or so

def login_required(f): # copied from finance 
    """
    Decorate routes to require login.

    https://flask.palletsprojects.com/en/latest/patterns/viewdecorators/
    """

    @wraps(f)
    def decorated_function(*args, **kwargs):
        if session.get("user_id") is None:
            return redirect("/login")
        return f(*args, **kwargs)

    return decorated_function

@app.route('/')
@login_required
def index():
    return render_template("main.html")

@app.route("/linefollower", methods=["GET", "POST"])
@login_required
def line():
    if request.method == "GET":
        #still need to do this but i had enough for today
        #get back the latest information about the robot
        return render_template("linefollower.html")


@app.route("/login", methods=["GET", "POST"])
def login():
    """Log user in"""

    # Forget any user_id
    session.clear()

    # User reached route via POST (as by submitting a form via POST)
    if request.method == "POST":
        # Ensure username was submitted
        if not request.form.get("username"):
            return render_template("error.html")

        # Ensure password was submitted
        elif not request.form.get("password"):
            return render_template("error.html")

        conn = get_db_conn()
        db = conn.cursor()

        # Query database for username
        rows = db.execute(
            "SELECT * FROM users WHERE username = ?", request.form.get("username")
        )

        # Ensure username exists and password is correct
        if len(rows) != 1 or not check_password_hash(
            rows[0]["hash"], request.form.get("password")
        ):
            return render_template("error.html")

        # Remember which user has logged in
        session["user_id"] = rows[0]["id"]

        db.commit()
        db.close()

        # Redirect user to home page
        return redirect("/")

    # User reached route via GET (as by clicking a link or via redirect)
    else:
        return render_template("login.html")


@app.route("/logout")
def logout():
    # Forget any user_id
    session.clear()

    # Redirect user to login form
    return redirect("/")

@app.route("/register", methods=["GET", "POST"])
def register():
    """Register user"""

    session.clear()

    if request.method == "POST":
        # ensure username was submitted
        if not request.form.get("username"):
            return render_template("error.html")

        # Ensure password was submitted
        elif not request.form.get("password"):
            return render_template("error.html")

        # Ensure password confirmation was submitted
        elif not request.form.get("confirmation"):
            return render_template("error.html")

        elif request.form.get("confirmation") != request.form.get("password"):
            return render_template("error.html")

        conn = get_db_conn()
        db = conn.cursor()

        rows = db.execute("SELECT * FROM users WHERE username = ?", request.form.get("username"))

        # Ensure username doesn't exist, had to do this way instead of try:except because we need the password hash when we try it
        if len(rows) != 0:
            return render_template("error.html")

        hash_pass = generate_password_hash(request.form.get("password"))

        try:
            user_id = db.execute("INSERT INTO users (username, hash) VALUES (?, ?)", request.form.get("username"), hash_pass)
        except:
            return render_template("error.html")
        db.commit()
        db.close()
        session["user_id"] = user_id

        return redirect("/")
    else:
        return render_template("register.html")