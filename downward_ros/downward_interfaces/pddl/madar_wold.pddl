(define (domain robot_bartender)
  (:requirements :strips :typing :equality :adl)

  (:types
    robot zone section object - object
    glass - object
    hand - object
  )

  (:predicates
    (at ?r - robot ?z - zone)
    (section-of ?s - section ?z - zone)
    (holding ?h - hand ?g - glass)
    (free-hand ?h - hand)
    (in-section ?g - glass ?s - section)
    (drink-prepared ?g - glass)
    (connected ?z1 - zone ?z2 - zone)  ; Connectivity between zones
  )

  ; Action: Move the robot from one zone to another
  (:action move-to-zone
    :parameters (?r - robot ?z1 - zone ?z2 - zone)
    :precondition (and
                    (at ?r ?z1)
                    (connected ?z1 ?z2))   ; Robot can only move between connected zones
    :effect (and
              (not (at ?r ?z1))
              (at ?r ?z2))
  )

  ; Action: Pick up two glasses simultaneously with both hands
  (:action pick-up-two-glasses
    :parameters (?r - robot
                ?h1 - hand ?g1 - glass ?s1 - section
                ?h2 - hand ?g2 - glass ?s2 - section
                ?z - zone)
    :precondition (and
                    (at ?r ?z)
                    (section-of ?s1 ?z)
                    (section-of ?s2 ?z)
                    (in-section ?g1 ?s1)
                    (in-section ?g2 ?s2)
                    (free-hand ?h1)
                    (free-hand ?h2)
                    (not (= ?g1 ?g2))
                    (not (= ?h1 ?h2)))
    :effect (and
              (holding ?h1 ?g1)
              (holding ?h2 ?g2)
              (not (free-hand ?h1))
              (not (free-hand ?h2))
              (not (in-section ?g1 ?s1))
              (not (in-section ?g2 ?s2)))
  )

  ; Action: Pick up a single glass with one hand
  ; Only allowed if the other hand is occupied
  (:action pick-up-glass
    :parameters (?r - robot ?h - hand ?g - glass ?s - section ?z - zone ?oh - hand)
    :precondition (and
                    (at ?r ?z)
                    (section-of ?s ?z)
                    (in-section ?g ?s)
                    (free-hand ?h)
                    (not (= ?h ?oh))
                    (not (free-hand ?oh))) ; Other hand is occupied
    :effect (and
              (holding ?h ?g)
              (not (free-hand ?h))
              (not (in-section ?g ?s)))
  )

  ; Action: Place two glasses from both hands into sections
  (:action place-two-glasses
    :parameters (?r - robot
                ?h1 - hand ?g1 - glass ?s1 - section
                ?h2 - hand ?g2 - glass ?s2 - section
                ?z - zone)
    :precondition (and
                    (holding ?h1 ?g1)
                    (holding ?h2 ?g2)
                    (at ?r ?z)
                    (section-of ?s1 ?z)
                    (section-of ?s2 ?z)
                    (not (= ?h1 ?h2))
                    (not (= ?g1 ?g2))
                    (or
                      (= ?z preparation-table)
                      (and
                        (drink-prepared ?g1)
                        (drink-prepared ?g2))))
    :effect (and
              (free-hand ?h1)
              (free-hand ?h2)
              (not (holding ?h1 ?g1))
              (not (holding ?h2 ?g2))
              (in-section ?g1 ?s1)
              (in-section ?g2 ?s2))
  )

  ; Action: Place a glass from a hand into a section
  ; At the preparation table, can place any glass
  (:action place-glass
    :parameters (?r - robot ?h - hand ?g - glass ?s - section ?z - zone)
    :precondition (and
                    (holding ?h ?g)
                    (at ?r ?z)
                    (section-of ?s ?z)
                    (or
                      (= ?z preparation-table)
                      (drink-prepared ?g))) ; At customer tables, glass must be prepared
    :effect (and
              (free-hand ?h)
              (not (holding ?h ?g))
              (in-section ?g ?s))
  )

  ; Action: Prepare a drink for a glass on the preparation table
  (:action prepare-drink-on-table
    :parameters (?r - robot ?g - glass ?s - section)
    :precondition (and
                    (at ?r preparation-table)
                    (section-of ?s preparation-table)
                    (in-section ?g ?s)
                    (not (drink-prepared ?g)))
    :effect (drink-prepared ?g)
  )

  ; Action: Prepare two drinks on the preparation table
  (:action prepare-two-drinks-on-table
    :parameters (?r - robot
                    ?g1 - glass ?s1 - section
                    ?g2 - glass ?s2 - section)
    :precondition (and
                    (at ?r preparation-table)
                    (free-hand left-hand)
                    (free-hand right-hand)
                    (section-of ?s1 preparation-table)
                    (section-of ?s2 preparation-table)
                    (in-section ?g1 ?s1)
                    (in-section ?g2 ?s2)
                    (not (drink-prepared ?g1))
                    (not (drink-prepared ?g2))
                    (not (= ?g1 ?g2)))
    :effect (and
              (drink-prepared ?g1)
              (drink-prepared ?g2))
  )

  ; Action: Pick up two prepared drinks from the preparation table
  (:action pick-up-two-drinks
    :parameters (?r - robot
                    ?h1 - hand ?g1 - glass ?s1 - section
                    ?h2 - hand ?g2 - glass ?s2 - section)
    :precondition (and
                    (at ?r preparation-table)
                    (section-of ?s1 preparation-table)
                    (section-of ?s2 preparation-table)
                    (in-section ?g1 ?s1)
                    (in-section ?g2 ?s2)
                    (free-hand ?h1)
                    (free-hand ?h2)
                    (drink-prepared ?g1)
                    (drink-prepared ?g2)
                    (not (= ?g1 ?g2))
                    (not (= ?h1 ?h2)))
    :effect (and
              (holding ?h1 ?g1)
              (holding ?h2 ?g2)
              (not (free-hand ?h1))
              (not (free-hand ?h2))
              (not (in-section ?g1 ?s1))
              (not (in-section ?g2 ?s2)))
  )

  ; Action: Pick up a single prepared drink from the preparation table
  ; Only allowed if the other hand is occupied
  (:action pick-up-drink
    :parameters (?r - robot ?h - hand ?g - glass ?s - section ?oh - hand)
    :precondition (and
                    (at ?r preparation-table)
                    (section-of ?s preparation-table)
                    (in-section ?g ?s)
                    (free-hand ?h)
                    (not (= ?h ?oh))
                    (not (free-hand ?oh)) ; Other hand is occupied
                    (drink-prepared ?g))
    :effect (and
              (holding ?h ?g)
              (not (free-hand ?h))
              (not (in-section ?g ?s)))
  )
)
