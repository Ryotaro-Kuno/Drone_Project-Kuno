import pickle
import sys #コマンドライン引数を使うための
import math

# START_POINT = (540, 999)
DISTANCE_LIMIT = 100

args = sys.argv #変数args[]にはコマンドラインに入力された値が順番に入る
#args[1]:Start,args[2]:Goal(0~31)、ただこれはstr型なのであとで変換する

with open('res/pathdata/opu.pickle', 'rb') as f:
    mapper = pickle.load(f) #距離のデータが入っているファイルを参照
e_path = mapper.paths #pathsデータを移してきた

nodes = mapper.default_targets #mapperのなかのdefault_targets(各チェックポイントの座標)を読み込んでいる
#print(e_path[forsearch][0]) #確認用出力
#print(len(nodes)) #確認用、nodesの長さは32(チェックポイントの数)

START_POINT = mapper.starting_point[0]


def euclid_distance(a, b):
    ax, ay = a  # ax = a[0]; ay = a[1]
    bx, by = b  # bx = b[0]; by = b[1]
    return math.sqrt((ax - bx)**2 + (ay - by)**2)


def find_near_points(s, g): #実際に見つけるメソッド(返り値：近い点を記録したリストnear_objects)
    path_use = e_path[(s, g)] #今回選ばれたチェックポイント間の経路
    near_objects = [] #近い点を記録するリスト
    for point in path_use[0]: #座標上で1マスずつ進んでいる
        for node in nodes:
            px, py = point[1], point[0]
            dis = euclid_distance((px, py), node)
            if dis < DISTANCE_LIMIT:
                near_objects.append(node) #near_objectに追加
    near_objects_set = list(dict.fromkeys(near_objects)) #重複を削除
    return near_objects_set


# チェックポイント番号(1-32)もしくはスタート地点(0)から、座標を取得
def get_node_from_number(num_str):
    num = int(num_str)
    if num == 0:
        return START_POINT #ドローンのスタート地点
    return nodes[num - 1]


def main():
    print("START")

    for i in range(len(args)-2): #0番目は別のデータなので、動く回数は(長さ)−1−1回
        start = get_node_from_number(args[i+1])
        goal  = get_node_from_number(args[i+2])
        near_objects_set = find_near_points(start, goal)
        print("Move {} ({} to {})".format(i+1, args[i+1], args[i+2]))
        print("Near point(s):", near_objects_set or 'Not Found.') #出力       

    print("END")


'''
def finding(a): #近い点を探すメソッド
    count = 0 #見つかった近い点を数える変数
    for i in e_path[a]: #座標上で1マスずつ進んでいる
        for j in len(nodes): #チェックポイントを探索
            dis = 1000000 #とても大きい数
            if nodes[j] != start and nodes[j] != end: #スタートとエンドは対象の点から外す
                dis = np.linalg.norm(e_path[a][i] - nodes[j]) #Numpyを用いて2点間の距離を計算
            if dis < 50: #取り敢えず距離50以下に設定
                count = count + 1
                print("object",count,nodes[j]) #出力
    if count == 0:
        print("No point was found.")
'''


'''
x = 0
print("start :",start,"goal :",goal)
for i in e_path: #持ってくる部分
    if start == distance_of[i][1]: #スタートが一致している
        if start == distance_of[i][2]: #ゴールが一致している
            x = i #何番目のデータか(distance_ofとe_pathは点の並び方が同じであるため)
            finding(x)
'''

'''
nodes_to_index:Dict[Node, int] = {}
for i, node in enumerate(nodes):
    nodes_to_index[node] = i + 1
'''


if __name__ == "__main__":
    main()
