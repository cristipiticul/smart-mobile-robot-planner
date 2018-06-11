using Graphics
using Cairo
using Tk

NR_OF_ROWS = 7
NR_OF_COLS = 8

win = Toplevel("Test", 800, 600)
c = Canvas(win)
pack(c, expand=true, fill="both")

ctx = getgc(c)
set_coordinates(ctx, 0, 0, 800, 600, 0, NR_OF_ROWS, 0, NR_OF_COLS)
set_source_rgb(ctx, 1, 1, 1)
paint(ctx)

# draw grid
for i=0:NR_OF_ROWS
    move_to(ctx, i, 0)
    line_to(ctx, i, NR_OF_COLS)
end
for i=0:NR_OF_COLS
    move_to(ctx, 0, i)
    line_to(ctx, NR_OF_ROWS, i)
end
