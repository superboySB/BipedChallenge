source /isaac-sim/setup_python_env.sh

# use isaac-sim python.sh
python() {
    /isaac-sim/python.sh "$@"
}
export -f python

pip() {
    /isaac-sim/python.sh -m pip "$@"
}
export -f pip

pip install -e /TongVerse/

python /TongVerse/app/main.py
