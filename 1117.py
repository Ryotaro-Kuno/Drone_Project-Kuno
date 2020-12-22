import pickle
import sys #コマンドライン引数を使うための
import math

# START_POINT = (528, 999)
DISTANCE_LIMIT = 100 #どれくらい近かったら検出するか
DECAY = 0.001279 #不確かさ計算における定数
DRONE_BATTERY = 3000

args = sys.argv #変数args[]にはコマンドラインに入力された値が順番に入る
#args[1]:Start,args[2]:Goal(0~31)、ただこれはstr型なのであとで変換する

with open('res/pathdata/opu.pickle', 'rb') as f:
    mapper = pickle.load(f) #距離のデータが入っているファイルを参照
e_path = mapper.paths #pathsデータを移してきた

nodes = mapper.default_targets #mapperのなかのdefault_targets(各チェックポイントの座標)を読み込んでいる
#print(e_path[forsearch][0]) #確認用出力
#print(len(nodes)) #確認用、nodesの長さは32(チェックポイントの数)
START_POINT = mapper.starting_point[0] #ドローンの出発点
nodes.insert(0, START_POINT) #出発点も含めたリスト

reset_points = [] #拠点に戻るサイクルごとの(何回目の移動で戻るか:充電の余裕)
cycle = 0
list_time = [0] * (len(nodes) - 1) #チェックポイントごとの経過時間

def euclid_distance(a, b):
    a_y, a_x = a #元データのx，yが逆になっている(ディープラーニングではそうすることも多いらしい)
    b_x, b_y = b
    return math.sqrt((a_x - b_x)**2 + (a_y - b_y)**2)

def find_near_points(s, g): #実際に見つけるメソッド(返り値：近い点を記録したリストnear_objects)
    path_use = e_path[(s, g)] #今回選ばれたチェックポイント間の経路
    near_object = [nodes.index(s)] #近い点を記録するリストnear_objectの略
    count = 0
    for c_path_use in path_use[0]: #for文の書き方。Bの中の要素一つ一つをAとして取り出す
        count += 1
        if count > DISTANCE_LIMIT * 1.1 and count < len(path_use[0]) - (DISTANCE_LIMIT * 1.1): #出発・到着点に近い場所では検出しない
            for i in range(len(nodes) - 1):
                dis = euclid_distance(c_path_use, nodes[i + 1]) #2点間の距離を計算
                if dis < DISTANCE_LIMIT:
                    near_object.append(i + 1) #near_objectに追加
    near_object.append(nodes.index(g))
    near_objects_set = list(dict.fromkeys(near_object)) #重複を削除
    return near_objects_set

def strtoint(l):
    l2 = []
    for i in range(len(l)-1):
        l2.append(int(l[i+1]))
    return l2

def get_node(num_str):
    return nodes[num_str]

def uncertainty_calculation(time): #不確かさの計算
    uncertainty = 1 - (1 / math.exp(DECAY * time))
    return uncertainty

def path_increase(sets): #寄り道すると経路がどれだけ増えるか
    pixel = 0 #増加した経路長
    poxel = e_path[get_node(sets[0]), get_node(sets[len(sets) - 1])][1] #もとの経路長
    if len(sets) > 2:
        for i in range(len(sets) - 1):
            pixel += e_path[get_node(sets[i]), get_node(sets[i+1])][1] #このループの合計が寄り道した経路長 
        pixel -= poxel 
    return pixel, poxel, pixel + poxel

def whether_go(sets_a):#増え幅が余裕を超えたら寄り道しない
    if path_increase(sets_a)[0] > reset_points[cycle][1]:
        sets_b = [sets_a[0], sets_a[len(sets_a) - 1]] #もとの経路を出力
        return sets_b
    else:
        reset_points[cycle][1] -= path_increase(sets_a)[0] #多く動く分だけ余裕を減らす
        return sets_a


def main():
    print("START")
    args1 = strtoint(args) #argsから0番目の要素を消す＋int型に
    #現状の余裕を計算
    yoyuu = DRONE_BATTERY

    for i in range(len(args1) - 1):
        moving = e_path[get_node(args1[i]), get_node(args1[i+1])][1]
        yoyuu -= moving
        if args1[i+1] == 0: 
            reset_points.append([i+1, yoyuu]) #拠点に戻った時点での充電残量
            yoyuu = DRONE_BATTERY #充電を満タンにする
        else:
            pass

        for i in range(len(list_time)): #全チェックポイントに経過時間を追加
            list_time[i] += moving
        
        list_time[args1[i+1] - 1] = 0 #到着点のみ経過時間リセット
    
    list_uncertainly = list(map(uncertainty_calculation, list_time)) 
    print(reset_points)
    print(list_time)


    cycle = 0
    #経路の改正(寄り道するかの判断)
    for i in range(len(args1)-1): #動く回数は(長さ)−1回
        start = get_node(args1[i])
        goal = get_node(args1[i+1])
        near_objects_set = find_near_points(start, goal)
        finally_path = whether_go(near_objects_set)

        if args1[i+1] == 0: #拠点に戻ったら次のサイクルへ
            cycle += 1
        
        print("Move {} ({} to {}))".format(i+1, args1[i],args1[i+1]))
        print("Path :",finally_path) #出力       

    
    print("END")

if __name__ == "__main__":
    main()



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
