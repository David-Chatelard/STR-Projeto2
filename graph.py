import matplotlib.pyplot as plt

executionTimeWander = [
    110651300,
    109384800,
    112165700,
    110100300,
    109323700,
    112154900,
    108952300,
    109099300,
    110899200,
    109247900,
    110801200,
    112193100,
    107854000,
    109099200,
    111361700,
    109652600,
    111554300,
    110489500,
    110348100,
    108761500,
    110054600,
    109808900,
    112034500,
    109976600,
    111963100,
    109004300,
    108270600,
    109969100,
    111986200,
    109102700,
    112104600,
    111388800,
    112145100,
    110100000,
    109159100,
    109919300,
    112102000,
    111117000,
    109736700,
    109744800,
    108325600,
    108756500,
    112090600,
    109458100,
    111436500,
    110101100,
    112167700,
    111227400,
    108732900,
    108394400,
    108197200,
    111111800,
    111932400,
    111395000,
    111247400,
    108296600,
    111653800,
    109098800,
    110251800,
    112113300,
    110235600,
    111688100,
    111470100,
    110099900,
    107951300,
    109678500,
    112425600,
    109776700,
    110061100,
    111365000,
    110091600,
    108943300,
    109099500,
    108861600,
    109674800,
    111038900,
    110396200,
    108935400,
    108925100,
    112103500,
    111762000,
    110822300,
    111085900,
    111942400,
    112107600,
    110100000,
    110114200,
    111727500,
    109099000,
    110910400,
    109050300,
    110573900,
    111574800,
    111894000,
    112120000,
    110084400,
    110793100,
    112114300,
    110157500,
    110108600,
    108050300,
    110042000,
    108013800,
    111637900,
    110114100,
    111620300,
    108098300,
    110918800,
    111038600,
    111958600,
    109682800,
    111109700,
    107984500,
    110643200,
    111940100,
    110206300,
    110102000,
    109972900,
    111101100,
    110105700,
    111684200,
    110100300,
    108962300,
    109948400,
    110726100,
    109099200,
    109105800,
    107838500,
    109020900,
    112106000,
    108599800,
    107924700,
    112120800,
    107313000,
    111311800,
    109966800,
    112110100,
    111525500,
    111120200,
    107790700,
    112112800,
    111452600,
    108779500,
    111226100,
    108708700,
    109731300,
    109101600,
    110847000,
    110159900,
    110868100,
    111102000,
    111099800,
    109929500,
    110228500,
    109353500,
    108099600,
    109961000,
    110430800,
    109255500,
    106922000,
    109999100,
    111011900,
    111589400,
    109925000,
    112103000,
    111007500,
    111108900,
    110966100,
    110868900,
    112120000,
    110855900,
    111100000,
    108992800,
    111107800,
    108778700,
    109152000,
    109098500,
    110890400,
    109101800,
    110571300,
    109404900,
    109045400,
    111098800,
    111100900,
    111164100,
    108008600,
    111114600,
    108784600,
    110682800,
    111103800,
    108811000,
    111100500,
    108576200,
    108655700,
    109098400,
    109739900,
    109745500,
    108541200,
    110390800,
    108865300,
    108843600,
    108105400,
    101116400,
    109926600,
    108207800,
    108783600,
    109984300,
    110337100,
    111100000,
    108879800,
    107902600,
    110013900,
    110780500,
    111104000,
    108889300,
    111103900,
    110105900,
    109940600,
    110742300,
    109217500,
    111125400,
    109598900,
    110404200,
    109638700,
    108674400,
    110663800,
    108414700,
    109619500,
    109876800,
    110099600,
    110308200,
    109724900,
    108866600,
    109979200,
    111087800,
    108156300,
    109057600,
    111111100,
    108739400,
    110072300,
    110271200,
    109668600,
    107600000,
    109098400,
    109931100,
    109098800,
]

print(f"executionTimeWander: {len(executionTimeWander)}")
# plt.title("Frequência do Tempo de Execução - Task Wander")
# plt.xlabel("Tempo de execução (ns)")
# plt.ylabel("Ocorrências")
# plt.hist(executionTimeWander, 10, rwidth=0.9)
# plt.show()

executionTimeUltrasonic = [
    126587500,
    112105700,
    112103400,
    109098900,
    110099700,
    112101800,
    112103000,
    108234900,
    223749100,
    111270000,
    222203500,
    141449500,
    223200,
    110100300,
    112102000,
    112102200,
    112103400,
    112101900,
    111009200,
    110099800,
    110101500,
    110677300,
    110100700,
    112102800,
    112101300,
    109462800,
    112102200,
    112102800,
    110100900,
    109163700,
    112102700,
    222202300,
    111822600,
    110605400,
    110471100,
    111802400,
    111518700,
    108097700,
    112101800,
    108233300,
    111605300,
    112102400,
    111548200,
    110100500,
    110100600,
    112104600,
    109099600,
    111102000,
    111105900,
    108097800,
    110207600,
    110101000,
    112102000,
    111602600,
    109831600,
    112101500,
    112101400,
    110100100,
    109098100,
    112101600,
    111101400,
    220199900,
    112102200,
    108739000,
    110779900,
    108623700,
    112101400,
    110102800,
    112156100,
    112101000,
    109099000,
    112101500,
    109125900,
    111100500,
    112102200,
    110100900,
    110100000,
    112105900,
    110500100,
    112165300,
    110068300,
    112049000,
    112101400,
    112101400,
    110099800,
    224204200,
    126617700,
    110101300,
    112101700,
    112102200,
    110100700,
    107712500,
    110100200,
    111101200,
    112104700,
    112103900,
    111927200,
    112100600,
    107096500,
    110098500,
    112101300,
    111101000,
    111089000,
    112102100,
    110709500,
    224204900,
    112101800,
    112102900,
    112101400,
    112103500,
    112101300,
    109099800,
    110101300,
    112101800,
    110100900,
    110100500,
    110635300,
    111302600,
    110099800,
    111200300,
    110098800,
    111099800,
    110100200,
    112102400,
    110102100,
    219199000,
    112101900,
    112102500,
    109676200,
    107640900,
    108601400,
    110099800,
    110100100,
    109387200,
    109418700,
    110099500,
    112101900,
    109243700,
    112101500,
    110979600,
    109099700,
    112102700,
    112111800,
    112103500,
    109602200,
    222202600,
    110099400,
    110099900,
    110100200,
    109101600,
    112101700,
]

print(f"executionTimeUltrasonic: {len(executionTimeUltrasonic)}")
# plt.title("Frequência do Tempo de Execução - Task Ultrasonic")
# plt.xlabel("Tempo de execução (ns)")
# plt.ylabel("Ocorrências")
# plt.hist(executionTimeUltrasonic, 10, rwidth=0.9)
# plt.show()

executionTimeColorSensor = [
    31027800,
    32030400,
    31027500,
    32029100,
    32028900,
    31028400,
    46042700,
    32027900,
    31028400,
    32028500,
    47151300,
    47043800,
    476100,
    32029000,
    31266200,
    31531700,
    32029400,
    31685400,
    32106500,
    455400,
]

print(f"executionTimeColorSensor: {len(executionTimeColorSensor)}")
# plt.title("Frequência do Tempo de Execução - Task Color Sensor")
# plt.xlabel("Tempo de execução (ns)")
# plt.ylabel("Ocorrências")
# plt.hist(executionTimeColorSensor, 10, rwidth=0.9)
# plt.show()

executionTimeRouteManager = [
    9364679400,
    8798090500,
    8786497200,
    9097256400,
    8613022600,
    8236801800,
    8173926800,
    9078289200,
    9131750900,
    8749137300,
    8676194400,
    8352578100,
    8151792700,
    8153053400,
    8726825400,
    8482827800,
    8449779500,
    11611997200,
    9784607000,
    8358977300,
    9947505500,
    8303643600,
    9157174100,
    8285697200,
    8465839400,
    8390819100,
    8997762700,
    8270727300,
    9020558800,
    8465213900,
    8387432900,
    9747028100,
    8144039900,
    8417252400,
    8776823000,
    8057803100,
    8069204700,
    8155647600,
    7997266600,
    8128828500,
    8010225200,
    8061704600,
    8819631500,
    8074532300,
    10345652900,
    9519265000,
    8800061100,
    8782699200,
    8440531100,
    7998762200,
    7997333900,
    8013730600,
    7981441900,
    7974899700,
    8026573700,
    7943904800,
]

print(f"executionTimeRouteManager: {len(executionTimeRouteManager)}")
# plt.title("Frequência do Tempo de Execução - Task Route Manager")
# plt.xlabel("Tempo de execução (ns)")
# plt.ylabel("Ocorrências")
# plt.hist(executionTimeRouteManager, 10, rwidth=0.9)
# plt.show()

executionTimeBlinkLeds = [
    407885000,
    413381700,
    414382600,
    418393100,
    414376700,
    409285700,
    417380100,
    410385900,
    418897700,
    413381100,
    423387400,
    416540200,
    413377400,
    407370700,
    410932800,
    414592800,
    408107900,
    416379700,
    410375900,
    413630900,
    416387800,
    409848300,
    410366300,
    408877100,
    410883200,
    410730600,
    411091100,
    411722400,
    409767000,
    414376800,
    411794100,
    412377200,
    410876600,
    411884900,
    408261100,
    409001800,
    412938200,
    414376300,
    411857000,
    413363900,
    415954800,
    411996600,
    406801600,
    413914200,
    411879500,
    407505100,
    409688400,
    411174300,
    407910800,
    412381400,
    411091500,
    412218400,
    409879300,
    412380100,
    410373700,
    411877600,
    409750500,
    410383200,
    408388300,
    407886600,
    410879200,
    412769100,
    409892900,
    406886600,
    409876700,
    408389800,
    410888000,
    411336900,
    409375500,
    412877000,
    409291400,
    406519200,
    414879400,
    409895900,
    414031900,
    414380500,
    414197700,
    409380100,
    409135600,
    408877500,
    410294000,
    413113900,
    407544100,
    412782300,
    411377800,
    406873100,
    408377300,
    413430100,
    411651300,
    413378300,
    407072600,
    410911600,
    415363100,
    408038000,
    410551500,
    412403500,
    408927500,
    406634700,
    405872800,
    405895200,
    413485800,
    411411500,
    411374100,
    411374200,
    410136100,
    411947000,
    408371800,
    409922900,
    410265900,
    411884900,
    410405400,
    410373200,
    411877800,
    415185700,
    412825400,
    410378400,
    409399900,
    409379500,
    410086600,
    408895800,
    404117500,
    409875400,
    406879800,
    408006600,
    410790500,
    413560100,
    409306900,
    406813000,
    406838700,
    408270400,
    411387700,
    410385300,
    411438500,
    411562700,
    412877000,
    414250000,
    410346300,
    412016600,
    409739700,
    414880500,
    411877800,
    410377800,
    413051500,
    410468600,
    408300000,
    410934400,
    409262900,
    410160900,
    409294600,
    407721100,
    408749600,
    411172900,
    409383900,
    412440600,
    409476800,
    410160000,
    407373100,
    411606700,
    413907700,
    407735000,
    415377900,
    409709600,
    409388100,
    411720700,
    415377200,
    406877300,
    413380400,
    414844300,
    410248200,
    409890500,
    408291000,
    407374500,
    401338100,
    410762300,
    414404700,
    409743300,
    406992800,
    410915900,
    409268000,
    411373200,
    406170100,
    406875500,
    408360200,
    412591200,
    413976500,
    408635200,
    413375500,
    408873900,
    408500600,
    410406100,
    415378600,
    412193500,
    411379400,
    409372500,
    413882300,
    409965600,
    410356700,
    407621500,
    410371900,
    410413800,
    411374000,
    415072200,
    413374600,
    409148100,
    409041300,
    409689600,
    407436400,
    406108600,
    411656800,
    407989100,
    411374400,
    408372700,
    410373400,
    411374700,
    409876200,
    410373200,
    412887600,
    411373300,
    408371300,
    412375000,
    412375200,
    410729100,
    412375600,
    412772000,
    422019700,
    407371300,
    418606500,
    412375100,
    406266900,
    413375900,
    407126000,
    418381700,
    410373500,
    412375400,
    412374900,
    406965100,
    408357000,
    409683100,
    414376700,
    408647200,
    403680600,
    409373600,
    407874000,
    407372400,
    407952300,
    408380000,
    407418300,
    412361300,
    408169400,
    409876000,
    412375900,
    408673500,
    407024200,
    410329900,
    409722800,
    408022100,
]

print(f"executionTimeBlinkLeds: {len(executionTimeBlinkLeds)}")
# plt.title("Frequência do Tempo de Execução - Task Blink LEDs")
# plt.xlabel("Tempo de execução (ns)")
# plt.ylabel("Ocorrências")
# plt.hist(executionTimeBlinkLeds, 10, rwidth=0.9)
# plt.show()
