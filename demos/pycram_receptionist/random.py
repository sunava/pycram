import random

def get_list(name: str, drink: str):
    id = random.randint(1,100)
    data_list = [name, drink, id]
    return data_list


if __name__ == '__main__':
    name = input("please enter name: ")
    drink = input("please enter drink: ")
    data_list = get_list(name, drink)
    print("hey " + data_list[0]+ " your drink is: " + data_list[1] + " and your id is: " + str(data_list[2]))