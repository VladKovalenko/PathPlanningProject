# PathPlanningProject

Коваленко Владислав, БМПИ-175. 


1.Входные данные.


   На вход алгоритм получает XML файл, содержащий:
   - Map, содержащае элементы 1(если данная ячейка является частью препядствия) или 0(через данную ячейку можно построить путь)
   - Тэги width и height, определяющие размеры поля
   - Тэг cellsize, определяющие размер одной ячейки
   - Тэги startx, starty, finishx, finishy, определяющие координаты начальной и конечной ячеек, между которыми и нужно найти кратчайший путь
   - Тэг metrictype, определяющий метрику(эвристику), используемую в алгоритме
   - Тэг searchtype, определяющий алгоритм поиска кратчайшего пути
   - Тэг hweight, определяющий вес H-value
   - Тэг breakingties
   - Тэги cutcorners, allowdiagonal, allowsqueeze, определяющие, можно ли срезать путь на углах, можно ли проходить по диагоналях, можно ли просачиваться через две клетки препядствия, находящихся по диагонали друг относительно друга
   
   
2.Описание алгоритма.

   Алгоритм, после получения XML файла начинает выполнение указанного алгоритма(Аstar, Dijkstra's, BFS, JPS, Thetastar), расчитывающий кратчайший путь между ячейкой start и ячейкой goal. 
   
   
3.Вывод.


   Вывод - файл XML, который дублирует входной файл, но содердит в Map путь, помеченный символом 'star', а также со значениями:
   - numberofsteps - количество вершин в списке close
   - nodescreated - количество вершин в списках close и open
   - length - длина пути
   - length_scaled
   - time - время работы алгоритма
