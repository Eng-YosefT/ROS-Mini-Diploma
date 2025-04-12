# ##################################################
# ملخص مفاهيم ROS 2 الأساسية
# ##################################################

# ##################################################
# 1. متغير ROS_LOCALHOST_ONLY
# ##################################################

# # الغرض: يخلي الـ ROS 2 يتكلم بس على الجهاز ده (localhost) وميتشافش من أجهزة تانية على الشبكة.
# # فايدته: مهم في الأماكن اللي فيها أجهزة كتير زي الفصول، عشان الروبوتات متدخلش في شغل بعضها لو بتستخدم نفس الـ topics.
# # الأمر (لينكس/ماك): export ROS_LOCALHOST_ONLY=1
# # عشان تفضل شغالة على طول: ضيف الأمر ده لملف الـ .bashrc بتاعك.
# # echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

# ##################################################
# 2. فهم الـ Nodes (العُقد)
# ##################################################

# # الـ ROS Graph: شبكة من مكونات ROS 2 اللي بتعالج البيانات مع بعض في نفس الوقت.
# # الـ Node: عبارة عن برنامج صغير مسؤول عن مهمة واحدة ومحددة (زي التحكم في عجلة، أو نشر بيانات حساس).
# # تواصل الـ Nodes: الـ Nodes بتتكلم مع بعض عن طريق: Topics, Services, Actions, Parameters.
# # أمر التشغيل: ros2 run <package_name> <executable_name>
# #   - بيشغل ملف تنفيذي (executable) من باكيدج معينة. مثال: ros2 run turtlesim turtlesim_node
# # أمر عرض الـ Nodes: ros2 node list
# #   - بيعرض أسماء كل الـ Nodes اللي شغالة دلوقتي.
# # أمر معلومات الـ Node: ros2 node info <node_name>
# #   - بيعرض تفاصيل node معينة زي مين بيستقبل منها (Subscribers) ومين بيبعت لها (Publishers) والخدمات (Services) والأكشنز (Actions).
# # إعادة التسمية (Remapping): ممكن تغير اسم الـ node أو خصائص تانية ليها وانت بتشغلها.
# #   - مثال: ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

# ##################################################
# 3. فهم الـ Topics (المواضيع)
# ##################################################

# # الـ Topic: زي قناة أو خط أتوبيس، الـ Nodes بتستخدمها عشان تبعت وتستقبل رسائل (بيانات) لبعضها.
# # النموذج: Publisher/Subscriber (ناشر/مشترك). ممكن node واحدة تنشر لكذا topic، وممكن كذا node تشترك في نفس الـ topic.
# # أداة التصور: rqt_graph
# #   - بتوريك شكل الـ Nodes والـ Topics والوصلات اللي بينهم في رسمة. شغلها بـ: rqt_graph
# # أمر عرض الـ Topics: ros2 topic list
# #   - بيعرض أسماء كل الـ Topics اللي شغالة.
# # أمر عرض الـ Topics وأنواعها: ros2 topic list -t
# #   - بيعرض أسماء الـ Topics ومعاها نوع الرسالة اللي بتستخدمها.
# # أمر استماع للـ Topic: ros2 topic echo <topic_name>
# #   - بيعرض البيانات اللي بتتبعت على topic معين أول بأول.
# # أمر معلومات الـ Topic: ros2 topic info <topic_name>
# #   - بيعرض عدد الـ Publishers والـ Subscribers ونوع الرسالة لـ topic معين.
# # أمر عرض هيكل الرسالة: ros2 interface show <message_type>
# #   - بيوريك الحقول (fields) اللي جوة نوع رسالة معين. مثال: ros2 interface show geometry_msgs/msg/Twist
# # أمر نشر على Topic: ros2 topic pub <topic_name> <message_type> '<arguments>'
# #   - بيبعت رسالة على topic معين من سطر الأوامر. الـ arguments بتتكتب بصيغة YAML.
# #   - مثال: ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"
# #   - خيارات:
# #     --once: يبعت رسالة مرة واحدة بس.
# #     --rate 1: يبعت الرسالة بمعدل 1 مرة في الثانية (1 Hz).
# #     -w N: يستنى لغاية ما يلاقي N مشتركين قبل ما يبعت.
# # أمر قياس معدل النشر: ros2 topic hz <topic_name>
# #   - بيقيس تقريباً عدد الرسائل اللي بتتبعت في الثانية على topic معين.
# # أمر قياس استهلاك الـ Bandwidth: ros2 topic bw <topic_name>
# #   - بيعرض كمية البيانات (bandwidth) اللي الـ topic ده بيستهلكها.
# # أمر البحث عن Topics بنوع معين: ros2 topic find <topic_type>
# #   - بيلاقي كل الـ topics اللي بتستخدم نوع رسالة معين.

# ##################################################
# 4. فهم الـ Services (الخدمات)
# ##################################################

# # الـ Service: طريقة تواصل تانية بين الـ Nodes، بتعتمد على نموذج طلب واستجابة (Request/Response).
# # الفرق عن الـ Topics: الـ Topics بتفضل تبعت بيانات على طول، لكن الـ Service بيبعت استجابة بس لما حد (Client) يطلب منه طلب (Request).
# # الاستخدام: مناسبة للعمليات اللي بتحصل مرة واحدة ومش مستمرة، زي طلب مسح الشاشة أو إعادة تعيين حاجة.
# # أمر عرض الـ Services: ros2 service list
# #   - بيعرض أسماء كل الـ Services اللي شغالة.
# # أمر عرض الـ Services وأنواعها: ros2 service list -t
# #   - بيعرض أسماء الـ Services ومعاها نوع الخدمة.
# # نوع الخدمة (Service Type): بيحدد شكل بيانات الطلب (Request) وشكل بيانات الاستجابة (Response). كل نوع خدمة ليه جزئين.
# # أمر عرض نوع الخدمة: ros2 service type <service_name>
# #   - بيعرض نوع خدمة معينة. مثال: ros2 service type /clear --> std_srvs/srv/Empty (نوع فاضي مبيبعتش ولا يستقبل بيانات).
# # أمر البحث عن Services بنوع معين: ros2 service find <type_name>
# #   - بيلاقي كل الـ services اللي بتستخدم نوع خدمة معين. مثال: ros2 service find std_srvs/srv/Empty
# # أمر عرض هيكل الخدمة: ros2 interface show <service_type>
# #   - بيوريك الحقول بتاعة الطلب (فوق ---) والحقول بتاعة الاستجابة (تحت ---). مثال: ros2 interface show turtlesim/srv/Spawn
# # أمر استدعاء خدمة: ros2 service call <service_name> <service_type> '<arguments>'
# #   - بيطلب خدمة معينة من سطر الأوامر. الـ arguments بتتكتب بصيغة YAML لو الخدمة محتاجاها.
# #   - مثال (خدمة مش محتاجة arguments): ros2 service call /clear std_srvs/srv/Empty
# #   - مثال (خدمة محتاجة arguments): ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, name: 'my_turtle'}"

# ##################################################
# 5. فهم الـ Parameters (المعاملات)
# ##################################################

# # الـ Parameter: قيمة إعدادات خاصة بـ node معينة. زي ظبط الإعدادات بتاعة برنامج. ممكن تكون رقم، عشري، بوليان (صح/غلط)، نص، أو قايمة.
# # أمر عرض الـ Parameters: ros2 param list
# #   - بيعرض كل الـ parameters لكل الـ nodes اللي شغالة.
# # أمر قراءة Parameter: ros2 param get <node_name> <parameter_name>
# #   - بيعرض نوع وقيمة parameter معين. مثال: ros2 param get /turtlesim background_g
# # أمر تعديل Parameter: ros2 param set <node_name> <parameter_name> <value>
# #   - بيغير قيمة parameter معين وهو شغال (التغيير ده مؤقت للجلسة الحالية بس). مثال: ros2 param set /turtlesim background_r 150
# # أمر حفظ الـ Parameters: ros2 param dump <node_name>
# #   - بيطبع كل الـ parameters الحالية بتاعة node معينة. ممكن تحفظ الخرج ده في ملف YAML.
# #   - مثال للحفظ في ملف: ros2 param dump /turtlesim > turtlesim.yaml
# # أمر تحميل الـ Parameters: ros2 param load <node_name> <parameter_file>
# #   - بيحمل قيم الـ parameters من ملف لـ node شغالة دلوقتي. (بعض الـ parameters ممكن تكون للقراءة فقط read-only ومتتغيرش بالطريقة دي).
# #   - مثال: ros2 param load /turtlesim turtlesim.yaml
# # أمر تشغيل Node بملف Parameters: ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
# #   - بيشغل node وهيستخدم القيم اللي في ملف الـ YAML كإعدادات أولية ليها (بيغير حتى الـ read-only parameters).
# #   - مثال: ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml

# ##################################################
# 6. فهم الـ Actions (الأفعال)
# ##################################################

# # الـ Action: نوع تواصل للمهام اللي بتاخد وقت طويل، ومحتاجة متابعة (feedback)، وممكن تتلغي (cancelable).
# # المكونات:
# #   1. Goal (الهدف): الـ Client بيبعته للـ Server عشان يبدأ المهمة.
# #   2. Feedback (المتابعة): الـ Server بيبعته للـ Client وهو شغال في المهمة عشان يعرفه وصل لفين.
# #   3. Result (النتيجة): الـ Server بيبعته للـ Client لما المهمة تخلص (بنجاح أو فشل).
# # النموذج: Client/Server. شبه الـ Services بس للمهام الطويلة ومعاها Feedback وإمكانية الإلغاء.
# # الإلغاء (Cancelation): الـ Client ممكن يلغي الـ Goal، أو الـ Server ممكن يلغيه (بيسموها Abort).
# # أمر معلومات الـ Node (للـ Actions): ros2 node info <node_name>
# #   - بيعرض الـ Action Servers (الـ node دي بتقدم الأكشن ده) والـ Action Clients (الـ node دي بتطلب الأكشن ده).
# # أمر عرض الـ Actions: ros2 action list
# #   - بيعرض أسماء كل الـ Actions اللي شغالة.
# # أمر عرض الـ Actions وأنواعها: ros2 action list -t
# #   - بيعرض أسماء الـ Actions ومعاها نوع الأكشن. مثال: /turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
# # أمر معلومات الـ Action: ros2 action info <action_name>
# #   - بيعرض مين الـ Clients ومين الـ Servers لأكشن معين.
# # أمر عرض هيكل الأكشن: ros2 interface show <action_type>
# #   - بيوريك هيكل الـ Goal (فوق أول ---)، وهيكل الـ Result (بين أول وتاني ---)، وهيكل الـ Feedback (تحت تاني ---).
# #   - مثال: ros2 interface show turtlesim/action/RotateAbsolute
# # أمر إرسال Goal: ros2 action send_goal <action_name> <action_type> '<goal_values>'
# #   - بيبعت طلب Goal لـ Action Server. الـ values بتتكتب بصيغة YAML.
# #   - مثال: ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
# #   - خيار عرض الـ Feedback: ضيف `--feedback` للأمر عشان تشوف رسائل المتابعة وهي بتوصل.
# #   - مثال مع Feedback: ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback

# ##################################################
# 7. استخدام rqt_console لعرض الـ Logs (السجلات)
# ##################################################

# # الأداة: rqt_console هي واجهة رسومية (GUI) عشان تشوف وتفحص رسائل الـ log اللي بتطلع من الـ Nodes.
# # الفايدة: بتجمع الرسائل، بتعرضها بشكل منظم، ممكن تفلترها، تحفظها، وتعيد تحميلها بعدين. أحسن من متابعتها في الترمينال العادي.
# # رسائل الـ Log: الـ Nodes بتستخدمها عشان تطلع معلومات عن الأحداث أو الحالة بتاعتها.
# # مستويات الـ Log (حسب الأهمية):
# #   - Fatal: خطأ قاتل، النظام غالباً هيقفل.
# #   - Error: خطأ كبير بيمنع النظام يشتغل صح.
# #   - Warn: تحذير، حاجة مش متوقعة حصلت بس ممكن النظام يكمل شغل.
# #   - Info: معلومات عادية عن حالة النظام (ده المستوى الافتراضي).
# #   - Debug: معلومات تفصيلية جداً عن كل خطوة.
# # المستوى الافتراضي (Default Level): هو Info، ومعناه إنك هتشوف رسائل الـ Info والـ Warn والـ Error والـ Fatal بس.
# # الفلترة: ممكن تفلتر الرسائل اللي بتظهر حسب المستوى (مثلاً تخفي الـ Info) أو حسب نص معين جواها.
# # تغيير المستوى الافتراضي عند التشغيل:
# #   - استخدم الأمر: ros2 run <package_name> <executable_name> --ros-args --log-level <LEVEL_NAME>
# #   - مثال (عشان تشوف الـ Warn والأعلى منه بس): ros2 run turtlesim turtlesim_node --ros-args --log-level WARN