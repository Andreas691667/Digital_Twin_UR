class TASK_DEFINITIONS:
    script_digital_out = b"""
        def fun(): \n
        \tset_digital_out(2,True) \n
        \tset_digital_out(3,True) \n
        end \n
        fun() \n
        """

    script_move = b"""
        def move(): \n
        \ti=0
        \twhile i < 5:  
        \t\tmovej([3.14, -1.5196582, 0.0001, -1.53589, 0.006, 0.02391101], a=0.3 , v=0.6) \n
        \t\tmovej([5, -1.5196582, -0.0959931, -1.53589, 0.006, 0.02391101], a=0.3 , v=0.6) \n
        \t\tmovej([0.004886922, -1.5196582, -0.0959931, -1.53589, 3.14, 0.02391101], a=0.3 , v=0.6) \n
        \t\ti=i+1 \n
        \tend
        end \n
        move() \n
        """