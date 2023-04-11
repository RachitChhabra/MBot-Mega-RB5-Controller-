# Tic Tac Toe Game
import numpy as np

# Define the game board
board = np.array([["-", "-", "-"],
                  ["-", "-", "-"],
                  ["-", "-", "-"]])

# Define a function to print the board
def print_board():
    print(board[0])
    print(board[1])
    print(board[2])

# Define a function to check if the game is over
def check_game_over():
    # Check rows for a win
    for row in range(3):
        if board[row][0] == board[row][1] == board[row][2] and board[row][0] != "-":
            print(f"{board[row][0]} wins!")
            return True
    # Check columns for a win
    for col in range(3):
        if board[0][col] == board[1][col] == board[2][col] and board[0][col] != "-":
            print(f"{board[0][col]} wins!")
            return True
    # Check diagonals for a win
    if board[0][0] == board[1][1] == board[2][2] and board[0][0] != "-":
        print(f"{board[0][0]} wins!")
        return True
    if board[0][2] == board[1][1] == board[2][0] and board[0][2] != "-":
        print(f"{board[0][2]} wins!")
        return True
    # Check for a tie
    if "-" not in board:
        print("Tie game!")
        return True
    return False

# Define a function to make a move
def make_move(player):
    row = int(input("Enter row (0, 1, or 2): "))
    col = int(input("Enter column (0, 1, or 2): "))
    if board[row][col] == "-":
        board[row][col] = player
        return True
    else:
        print("That space is already occupied!")
        return False

# Play the game
print("Welcome to Tic Tac Toe!")
print_board()
game_over = False
while not game_over:
    # Player 1 move
    while True:
        print("Player 1's turn (X)")
        if make_move("X"):
            break
    print_board()
    game_over = check_game_over()
    if game_over:
        break
    # Player 2 move
    while True:
        print("Player 2's turn (O)")
        if make_move("O"):
            break
    print_board()
    game_over = check_game_over()
