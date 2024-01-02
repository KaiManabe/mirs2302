import authentication as auth
import sys
import order_mng as om
import robot_state_publisher as rsp

o = om.order_manager()

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        if auth.auth(sys.argv[1]) == True:
            MODE = "PICKUP"
            order = o.get_order("STATUS", "WAITING_FOR_" + MODE)
            if type(order) == int and order == -1:
                MODE = "RECEIVE"
                order = o.get_order("STATUS", "WAITING_FOR_" + MODE)
                if type(order) == int and order == -1:
                    print("[ERR][authentication.py] : 集荷・受取待ち状態のオーダが存在しません")
                    sys.exit(-1)

            order = order.iloc[0]
            if MODE == "PICKUP":
                o.modify_order(order["ID"], "STATUS", "PICKED_UP")
                rsp.update("ROBOT_STATUS", "WAITING")
                print(True, end = "")
            else:
                o.modify_order(order["ID"], "STATUS", "RECEIVED")
                rsp.update("ROBOT_STATUS", "WAITING")
                print(True, end = "")