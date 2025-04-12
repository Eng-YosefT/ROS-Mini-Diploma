# ##################################################
# أهم أوامر ROS 2 الأساسية
# ##################################################

# --- التعامل مع الـ Nodes (العُقد) ---

# تشغيل Node (ملف تنفيذي من باكيدج)
# ros2 run <package_name> <executable_name>
# مثال:
# ros2 run turtlesim turtlesim_node

# عرض كل الـ Nodes اللي شغالة حالياً
# ros2 node list

# عرض معلومات تفصيلية عن Node معينة (Subscribers, Publishers, Services, Actions)
# ros2 node info <node_name>
# مثال:
# ros2 node info /turtlesim

# تشغيل Node مع تغيير اسمها (Remapping)
# ros2 run <package_name> <executable_name> --ros-args --remap __node:=<new_node_name>
# مثال:
# ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

# --- التعامل مع الـ Topics (المواضيع) ---

# عرض كل الـ Topics اللي شغالة
# ros2 topic list

# عرض الـ Topics وأنواع الرسائل بتاعتها
# ros2 topic list -t

# عرض البيانات اللي بتتبعت على Topic معين (استماع)
# ros2 topic echo <topic_name>
# مثال:
# ros2 topic echo /turtle1/cmd_vel

# عرض معلومات عن Topic معين (عدد الناشرين والمشتركين ونوع الرسالة)
# ros2 topic info <topic_name>
# مثال:
# ros2 topic info /turtle1/cmd_vel

# عرض هيكل (الحقول) بتاع نوع رسالة معين
# ros2 interface show <message_type>
# مثال:
# ros2 interface show geometry_msgs/msg/Twist

# نشر (إرسال) رسالة على Topic معين من سطر الأوامر
# ros2 topic pub <topic_name> <message_type> '<arguments_in_yaml_format>'
# مثال لنشر مستمر (بمعدل 1 Hz افتراضي):
# ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"
# مثال لنشر مرة واحدة فقط:
# ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
# مثال للنشر بمعدل معين (مثلاً 5 مرات في الثانية):
# ros2 topic pub --rate 5 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# قياس معدل الرسائل (كم رسالة في الثانية تقريباً) على Topic
# ros2 topic hz <topic_name>
# مثال:
# ros2 topic hz /turtle1/pose

# البحث عن كل الـ Topics اللي بتستخدم نوع رسالة معين
# ros2 topic find <type_name>
# مثال:
# ros2 topic find geometry_msgs/msg/Twist

# قياس استهلاك الـ Bandwidth بتاع Topic معين
# ros2 topic bw <topic_name>
# مثال:
# ros2 topic bw /turtle1/pose

# --- التعامل مع الـ Services (الخدمات) ---

# عرض كل الـ Services اللي شغالة
# ros2 service list

# عرض الـ Services وأنواعها
# ros2 service list -t

# عرض نوع Service معين
# ros2 service type <service_name>
# مثال:
# ros2 service type /clear

# البحث عن كل الـ Services اللي بتستخدم نوع معين
# ros2 service find <type_name>
# مثال:
# ros2 service find std_srvs/srv/Empty

# عرض هيكل (الطلب والاستجابة) بتاع نوع Service معين
# ros2 interface show <service_type>
# مثال:
# ros2 interface show turtlesim/srv/Spawn

# استدعاء (طلب) Service معين من سطر الأوامر
# ros2 service call <service_name> <service_type> '<arguments_in_yaml_format_if_needed>'
# مثال لخدمة مش محتاجة arguments:
# ros2 service call /clear std_srvs/srv/Empty
# مثال لخدمة محتاجة arguments:
# ros2 service call /spawn turtlesim/srv/Spawn "{x: 5, y: 1, theta: 0.5, name: 'my_new_turtle'}"

# --- التعامل مع الـ Parameters (المعاملات) ---

# عرض كل الـ Parameters لكل الـ Nodes اللي شغالة
# ros2 param list

# قراءة قيمة ونوع Parameter معين لـ Node معينة
# ros2 param get <node_name> <parameter_name>
# مثال:
# ros2 param get /turtlesim background_g

# تعديل قيمة Parameter معين لـ Node معينة (التغيير مؤقت للجلسة الحالية)
# ros2 param set <node_name> <parameter_name> <value>
# مثال:
# ros2 param set /turtlesim background_r 150

# حفظ كل الـ Parameters الحالية لـ Node معينة في ملف YAML
# ros2 param dump <node_name> > <your_filename.yaml>
# مثال:
# ros2 param dump /turtlesim > turtlesim_settings.yaml

# تحميل قيم Parameters من ملف YAML إلى Node شغالة حالياً (قد لا يغير الـ read-only)
# ros2 param load <node_name> <your_filename.yaml>
# مثال:
# ros2 param load /turtlesim turtlesim_settings.yaml

# تشغيل Node مع تحميل قيم Parameters من ملف YAML (بيغير حتى الـ read-only)
# ros2 run <package_name> <executable_name> --ros-args --params-file <your_filename.yaml>
# مثال:
# ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim_settings.yaml

# --- التعامل مع الـ Actions (الأفعال) ---

# عرض كل الـ Actions اللي شغالة
# ros2 action list

# عرض الـ Actions وأنواعها
# ros2 action list -t

# عرض معلومات عن Action معين (مين الـ Client ومين الـ Server)
# ros2 action info <action_name>
# مثال:
# ros2 action info /turtle1/rotate_absolute

# عرض هيكل (Goal, Result, Feedback) بتاع نوع Action معين
# ros2 interface show <action_type>
# مثال:
# ros2 interface show turtlesim/action/RotateAbsolute

# إرسال طلب Goal لـ Action معين من سطر الأوامر
# ros2 action send_goal <action_name> <action_type> '<goal_values_in_yaml_format>'
# مثال:
# ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 3.14}"
# مثال مع طلب عرض رسائل الـ Feedback:
# ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback

# --- أدوات مساعدة ---

# تشغيل واجهة رسومية لعرض الـ Nodes والـ Topics والوصلات بينهم
# rqt_graph

# تشغيل واجهة رسومية لعرض وتصفية رسائل الـ Log
# rqt_console

# تشغيل Node مع تحديد مستوى الـ Log الافتراضي (مثلاً WARN عشان تشوف التحذيرات والأخطاء بس)
# ros2 run <package_name> <executable_name> --ros-args --log-level <LEVEL_NAME>
# مثال:
# ros2 run turtlesim turtlesim_node --ros-args --log-level WARN