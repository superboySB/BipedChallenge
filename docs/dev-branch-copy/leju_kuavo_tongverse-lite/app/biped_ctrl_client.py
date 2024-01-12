import json
import time

from tongverselite.tcp import PersistentTcpClient, json2bin


class BipedCtrlClient(PersistentTcpClient):
    def get_command(self, msg):
        data_bin = json2bin(msg)
        return json.loads(self.send(data_bin).decode("ascii"))


def main():
    client = BipedCtrlClient(ip="10.1.100.217", port=8800)

    msg = {"joint_state": {"elbow": 1.00}}

    n_times = 100
    start_t = time.time()
    for _ in range(n_times):
        cmd = client.get_command(msg)
        print(cmd)
    print(f"frequency: {n_times / (time.time() - start_t)}")

    # end of connection
    client.close()


if __name__ == "__main__":
    main()
