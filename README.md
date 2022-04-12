# suturo_knowledge

## Wichtige Desingentscheidungen und Prolog patterns:
1. Bei `tell(is_at(name,...))` wird vom Namen am Anfang Zahlen und Minusse entfernt. Damit weniger Bugs auftreten, sind die Knowledge Objektklassen angepasst und store_object_info_server.py passt die Namen entsprechend an.
   Damit `marker_objectstate.py` den Klassennamen einfach erkennen kann, werden auch alle Unterstriche aus dem Klassennamen entfernt.  
   Zum einfacheren Umgang macht Knowledge zusätzlich den ersten Buchstaben groß.  
   Also `re.sub(r"^[0-9-_]+|_","",name).capitalize()` in Python.  
2. ifs in Prolog sehen so aus: `condition -> if_true; if_false`.  
   Das beudeutet aber, das wenn man loggen will, wenn ein teil fehlschlägt brauch man folgendes Konstrukt:  
   `(<query> -> true; ros_info("<query> failed"))`

## Altlasten, bei denen ich die Designentscheidungen noch nicht ganz verstehe:
- `marker_objectstate.py` republisht alle `/visualization_marker_array` als Objekte auf `/object_state`.
  Warum wird das nicht direkt in prolog gemacht, ohne dass man einen extra Subscriber braucht, besonders da dieser möglicherweise auch Marker von anderen weiterleitet?  
  Wird möglicherweise ausgenutzt, dass rviz die Marker bewegt?
