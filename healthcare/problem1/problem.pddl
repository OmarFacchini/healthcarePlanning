(define (problem problem1_general)
    (:domain healthcare-problem1_general)

    (:objects

        ; no definition of number of patient, start simple with just 2
        patient1 patient2 - patient

        ; problem states we have 1 instance per type of robot
        carrier1  - robot_carrier
        escorter1 - robot_accompany

        ; no definition of number of boxes, start simple with just 2
        box1 box2 - box

        ; no definition on number of items, just indirectly stated there can be many of the same item (5 scalpels)
        ; used in emergency_department
        defibrillator1 defibrillator2 - defibrillator
        oxygen1 oxygen2 - oxygen

        ; used in intensive_care_department
        ventilator1 ventilator2 - ventilator
        feeding_pump1 feeding_pump2 - feeding_pump

        ; used in progressive_care department
        IV_pump1 IV_pump2 - IV_pump
        ECG_machine1 ECG_machine2 - ECG_machine

        ; used in medical_department
        scalpel1 scalpel2 - scalpel
        forcep1 forcep2 - forcep

        ; the constant locations indicated in the problem, starting point of escorter and carrier respectively
        entrance central_warehouse - location

        ; extra locations to make the example more realistic
        ; (avoid having the entrance directly connected to a medical section)
        hallway waiting_room - location

        ; department(location) 1 and the rooms(units) inside of it
        emergency_department - location
        emergency_trauma emergency_psychiatric emergency_pediatric - unit

        ; department(location) 2 and the rooms(units) inside of it
        intensive_care_department - location
        intensive_surgical intensive_cardiac intensive_pediatric - unit
        
        ; department(location) 3 and the rooms(units) inside of it
        progressive_care_department - location
        progressive_surgical progressive_cardiac progressive_pediatric - unit
        
        ; department(location) 3 and the rooms(units) inside of it
        medical_department - location
        medical_post_surgical medical_orthopedics medical_oncology - unit
    
    )

    (:init
        ; entrance only connected to main way of walking
        (are_connected entrance hallway)

        ; central_warehouse only connected to hallway
        (are_connected central_warehouse hallway)
        
        ; hallway connected to basically every location
        (are_connected hallway entrance)
        (are_connected hallway waiting_room)
        (are_connected hallway central_warehouse)
        (are_connected hallway emergency_department)
        (are_connected hallway intensive_care_department)
        (are_connected hallway progressive_care_department)
        (are_connected hallway medical_department)

        ; waiting_room only connected and hallway
        (are_connected waiting_room hallway)

        ; the departmnts are only connected to the hallway
        (are_connected emergency_department hallway)
        (are_connected intensive_care_department hallway)
        (are_connected progressive_care_department hallway)
        (are_connected medical_department hallway)

        ; assign each unit to a department, only 
        (at emergency_trauma emergency_department)
        (at emergency_psychiatric emergency_department)
        (at emergency_pediatric emergency_department)

        (at intensive_surgical intensive_care_department)
        (at intensive_cardiac intensive_care_department)
        (at intensive_pediatric intensive_care_department)
        
        (at progressive_surgical progressive_care_department)
        (at progressive_cardiac progressive_care_department)
        (at progressive_pediatric progressive_care_department)
        
        (at medical_post_surgical medical_department)
        (at medical_orthopedics medical_department)
        (at medical_oncology medical_department)


        ; escorter and patient start at the entrance
        (at escorter1 entrance)
        (at patient1 entrance)
        (at patient2 entrance)

        ; set patient to needing to be escorted
        (needs_escorting patient1)
        (needs_escorting patient2)

        ; set the patient and escorter to be free
        (patient_free patient1)
        (patient_free patient2)
        (escorter_free escorter1)

        ; set location that the patients want to reach
        (needs_to_reach patient1 emergency_trauma)
        (needs_to_reach patient2 intensive_cardiac)

        ;======================================================================================= DO NOT TOUCH ABOVE, IT WORKS


        ; carrier and boxes start in the central_warehouse (defined in initial condition of the problem)
        (at carrier1 central_warehouse)
        (carrier_free carrier1)

        (at box1 central_warehouse) 
        (at box2 central_warehouse)

        ; specify that the boxes are empty (otherwise no item will be inserted in them)
        (is_empty box1)
        (is_empty box2)

        ; and free
        (box_free box1)
        (box_free box2)

        ; start simple with all items in the warehouse too (defined in initial condition of the problem)
        ; this emulates a clean start of the day
        ; later on could have them randomly spread in units to simulate the middle of the day
        (at defibrillator1 central_warehouse)
        (at defibrillator2 central_warehouse)
        (at oxygen1 central_warehouse)
        (at oxygen2 central_warehouse)

        (at ventilator1 central_warehouse)
        (at ventilator2 central_warehouse)
        (at feeding_pump1 central_warehouse)
        (at feeding_pump2 central_warehouse)

        (at IV_pump1 central_warehouse)
        (at IV_pump2 central_warehouse)
        (at ECG_machine1 central_warehouse)
        (at ECG_machine2 central_warehouse)

        (at scalpel1 central_warehouse)
        (at scalpel2 central_warehouse)
        (at forcep1 central_warehouse)
        (at forcep2 central_warehouse)

        ; no need to use the inside/contains predicates to indicate that the items are not in any box
        ; since everything that is not specified is defaulted to false, which is exactly our case


        ; indicate which unit needs which item
        ; we start simple with each unit only requiring only 1 item and with no overlaps
        ; eg: defibrillators only used in emergency_department, not in others
        ; also not need more items than we have
        ; eg: have 2 defibrillators, we will not have 3 units request defibrillators
        ; make us need more that 1 item in the same unit
        (need_item emergency_trauma defibrillator1)
        (need_item emergency_trauma defibrillator2)
        (need_item emergency_psychiatric oxygen1)
        (need_item emergency_pediatric oxygen2)

        ; generale simple case
        (need_item intensive_surgical ventilator1)
        (need_item intensive_cardiac feeding_pump1)
        (need_item intensive_pediatric ventilator2)

        ; another simple case
        (need_item progressive_surgical IV_pump1)
        (need_item progressive_cardiac IV_pump2)
        (need_item progressive_pediatric ECG_machine1)


        ; assume no units of medical_department need any item, as per part of the goal defined by the problem
    )

    (:goal 
        (and 
            ; check that patients reached the needed unit
            (has_reached patient1 emergency_trauma)
            (has_reached patient2 intensive_cardiac)

            ; make sure the patients was also let go from the escorter
            (patient_free patient1)
            (patient_free patient2)

            ; also make sure the escorter is free as there are no more patients to escort
            (escorter_free escorter1)

            ; check for any item of that type that is required by the uni
            ; this allows eg: emergency_trauma unit, which needs a defibrillator, to accept 
            ; either defibrillator1 or defibrillator2
            (exists (?d - defibrillator) (has_item emergency_trauma ?d))
            (exists (?ox - oxygen) (has_item emergency_psychiatric ?ox))
            (exists (?ox - oxygen) (has_item emergency_pediatric ?ox))
            (exists (?v - ventilator) (has_item intensive_surgical ?v))
            (exists (?fp - feeding_pump) (has_item intensive_cardiac ?fp))
            (exists (?v - ventilator) (has_item intensive_pediatric ?v))
            (exists (?ip - IV_pump) (has_item progressive_surgical ?ip))
            (exists (?ip - IV_pump) (has_item progressive_cardiac ?ip))
            (exists (?ec - ECG_machine) (has_item progressive_pediatric ?ec))

            ; this was another possible solution which i first came up with
            ; but is really ugly and does not scale with more items of a type
            ; kept here just to show another possible solution, might remove it later
            ;(or (has_item emergency_trauma defibrillator1)
            ;    (has_item emergency_trauma defibrillator2)
            ;)
            ;(or (has_item emergency_psychiatric oxygen1)
            ;    (has_item emergency_psychiatric oxygen1)
            ;)
            ;(or (has_item emergency_pediatric oxygen2)
            ;    (has_item emergency_pediatric oxygen1)
            ;)
            ;(or (has_item intensive_surgical ventilator1)
            ;    (has_item intensive_surgical ventilator2)
            ;)
            ;(or (has_item intensive_cardiac feeding_pump1)
            ;    (has_item intensive_cardiac feeding_pump2)
            ;)
            ;(or (has_item intensive_pediatric ventilator2)
            ;    (has_item intensive_pediatric ventilator1)
            ;)

            ;(or (has_item progressive_surgical IV_pump1)
            ;    (has_item progressive_surgical IV_pump2)
            ;)
            ;(or (has_item progressive_cardiac IV_pump2)
            ;    (has_item progressive_cardiac IV_pump1)
            ;)

            ;(or (has_item progressive_pediatric ECG_machine1)
            ;    (has_item progressive_pediatric ECG_machine2)
            ;)
      )
    )
)