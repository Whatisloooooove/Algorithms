В этом алгоритме я исползую вектор векторов для хранения всех векторов
Далее я попарно в функции MakeMergeAll соединия их в один вектор используя функцию Merge из лекции
За счёт того , что я склеиваю их получается сложность log K и с каждым из них я делаю Merge (O(S))
Итоговая сложность: O(SlogK)
